import argparse
import os
import signal
import sys
import threading
import time
import yaml

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, "..", "..")
sys.path.insert(0, project_root)

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

from duckiebot.camera_driver import CameraDriver
from duckiebot.led_driver import LEDDriver
from duckiebot.wheel_driver import DaguWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from launcher.ports import find_available_port
from servers.common import shutdown_cleanup, suppress_http_logs
from servers.templates.project import get_template as HTML_TEMPLATE
from tasks.project.packages.optimal_path import dijkstra

app = Flask(__name__)
app.static_folder = os.path.join(project_root, "static")
camera = None
wheels = None
leds = None
stop_event = threading.Event()

current_node = 1
start_direction = "E"  # Default robot heading (change via UI grid picker)
goal_node = 3
_manual_mode = True
_navigation_thread = None
_navigation_stop = threading.Event()
_mode_lock = threading.Lock()
_camera_frame_condition = threading.Condition()
_camera_latest_frame = None
_camera_latest_time = 0.0
_camera_frame_sequence = 0
_camera_capture_thread = None
keys_pressed = {"up": False, "down": False, "left": False, "right": False}
keys_lock = threading.Lock()
_keys_last_update = time.time()
current_speeds = {"left": 0.0, "right": 0.0}


class _ResilientWheels:
    """Retry transient Jetson I2C NACKs without killing navigation."""

    def __init__(self, physical_wheels):
        self._wheels = physical_wheels
        self._lock = threading.Lock()
        self._consecutive_io_failures = 0

    def set_wheels_speed(self, left, right):
        attempts = 3 if abs(left) < 1e-6 and abs(right) < 1e-6 else 2
        with self._lock:
            for attempt in range(attempts):
                try:
                    result = self._wheels.set_wheels_speed(left, right)
                    if self._consecutive_io_failures:
                        print("[Wheels] I2C communication recovered", flush=True)
                    self._consecutive_io_failures = 0
                    return result
                except OSError as exc:
                    if getattr(exc, "errno", None) != 121:
                        raise
                    self._consecutive_io_failures += 1
                    if attempt + 1 < attempts:
                        time.sleep(0.02)
                        continue
                    if (
                        self._consecutive_io_failures == 1
                        or self._consecutive_io_failures % 20 == 0
                    ):
                        print(
                            f"[Wheels] Transient I2C error after {attempts} attempts: {exc}",
                            flush=True,
                        )
                    return None

    def __getattr__(self, name):
        return getattr(self._wheels, name)

class _LatestFrameCamera:
    """Camera-compatible view backed by the permanent capture thread."""

    def __init__(self, physical_camera):
        self._camera = physical_camera
        self._last_sequence = -1

    def read(self):
        ok, frame, sequence, _timestamp = _latest_camera_frame(
            after_sequence=self._last_sequence,
            timeout=0.5,
        )
        if ok:
            self._last_sequence = sequence
        return ok, frame

    def __getattr__(self, name):
        return getattr(self._camera, name)


def _latest_camera_frame(after_sequence=None, timeout=0.0):
    with _camera_frame_condition:
        if (
            after_sequence is not None
            and _camera_frame_sequence <= after_sequence
            and not stop_event.is_set()
        ):
            _camera_frame_condition.wait(timeout)
        if _camera_latest_frame is None:
            return False, None, _camera_frame_sequence, _camera_latest_time
        return (
            True,
            _camera_latest_frame.copy(),
            _camera_frame_sequence,
            _camera_latest_time,
        )


def _camera_capture_loop():
    global _camera_latest_frame, _camera_latest_time, _camera_frame_sequence
    while not stop_event.is_set():
        physical_camera = camera
        if physical_camera is None:
            stop_event.wait(0.05)
            continue
        try:
            ok, frame = physical_camera.read()
        except Exception as exc:
            print(f"[CameraCapture] Read error: {exc}", flush=True)
            stop_event.wait(0.05)
            continue
        if not ok or frame is None:
            stop_event.wait(0.01)
            continue
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]
        with _camera_frame_condition:
            _camera_latest_frame = frame.copy()
            _camera_latest_time = time.time()
            _camera_frame_sequence += 1
            _camera_frame_condition.notify_all()


def _start_camera_capture_thread():
    global _camera_capture_thread
    if _camera_capture_thread is None or not _camera_capture_thread.is_alive():
        _camera_capture_thread = threading.Thread(
            target=_camera_capture_loop,
            daemon=True,
            name="ProjectCameraCapture",
        )
        _camera_capture_thread.start()

_DANCE_COLORS = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 1.0],
    [1.0, 0.0, 1.0],
]
maneuver_thread = None
maneuver_stop = threading.Event()


def start_maneuver(fn, *args):
    global maneuver_thread, maneuver_stop
    maneuver_stop.set()
    if maneuver_thread and maneuver_thread.is_alive():
        maneuver_thread.join(timeout=1.5)
    maneuver_stop = threading.Event()
    maneuver_thread = threading.Thread(
        target=fn, args=(*args, maneuver_stop), daemon=True
    )
    maneuver_thread.start()


def dance(duration_sec, stop_ev):
    print(f"[Dance] Starting for {duration_sec:.1f}s")
    duration = float(np.clip(duration_sec, 0.5, 10.0))

    dance_colors = [
        [1.0, 0.0, 0.0],  # red
        [0.0, 0.0, 1.0],  # blue
        [0.0, 1.0, 0.0],  # green
    ]

    # Stop and pause before dancing (matching agent behaviour)
    if wheels:
        wheels.set_wheels_speed(0.0, 0.0)
    time.sleep(2.0)

    end_time = time.time() + duration
    step = 0
    while not stop_ev.is_set() and time.time() < end_time:
        l, r = (0.8, -0.8) if step % 2 == 0 else (-0.8, 0.8)
        if wheels:
            wheels.set_wheels_speed(l, r)
        if leds:
            try:
                color = dance_colors[step % len(dance_colors)]
                for led in (0, 2, 3, 4):
                    leds.set_rgb(led, color)
            except Exception:
                pass
        time.sleep(0.1)
        step += 1

    # Clean up
    if wheels:
        wheels.set_wheels_speed(0.0, 0.0)
    if leds:
        try:
            leds.all_off()
        except Exception:
            pass
    print("[Dance] Done")


# ── Background overlay thread ────────────────────────────────────────────────
# Runs detection in a separate thread so generate_frames() never blocks.
_overlay_frame = [None]
_overlay_frame_time = [0.0]
_last_video_frame = [None]
_overlay_lock = threading.Lock()
_overlay_thread = None


def _overlay_loop():
    import tasks.project.packages.agent as agent_mod
    preview_camera = _LatestFrameCamera(camera)
    last_debug_id = None

    while True:
        try:
            # Navigation owns the camera while the agent is running; display only
            # the agent-produced debug frame so the overlay thread does not steal reads.
            if not _manual_mode:
                current_debug = agent_mod.debug_frame
                current_debug_id = id(current_debug) if current_debug is not None else None
                if current_debug is not None and current_debug_id != last_debug_id:
                    with _overlay_lock:
                        _overlay_frame[0] = current_debug
                        _overlay_frame_time[0] = time.time()
                    last_debug_id = current_debug_id
                time.sleep(0.02)
                continue

            # Manual mode owns the live camera preview.
            if camera is None:
                time.sleep(0.05)
                continue

            ok, frame = preview_camera.read()
            if not ok or frame is None:
                time.sleep(0.033)
                continue

            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]

            try:
                from tasks.project.packages.agent import build_debug_frame, detect_red_line
                from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
                    detect_lane_markings,
                )

                mask_y_f, mask_w_f = detect_lane_markings(frame)
                _, mask_red_f = detect_red_line(frame)
                display = build_debug_frame(
                    raw_bgr=frame,
                    mask_yellow=(mask_y_f * 255).astype(np.uint8),
                    mask_white=(mask_w_f * 255).astype(np.uint8),
                    mask_red=mask_red_f,
                    state="manual",
                    sub=None,
                    error=0.0,
                )
            except Exception:
                display = frame

            with _overlay_lock:
                _overlay_frame[0] = display
                _overlay_frame_time[0] = time.time()

        except Exception:
            time.sleep(0.05)


def _start_overlay_thread():
    global _overlay_thread
    if _overlay_thread is None or not _overlay_thread.is_alive():
        _overlay_thread = threading.Thread(
            target=_overlay_loop, daemon=True, name="ProjectOverlay"
        )
        _overlay_thread.start()


def generate_frames():
    placeholder = None
    while True:
        try:
            now = time.time()
            with _overlay_lock:
                display = _overlay_frame[0]
                overlay_age = now - _overlay_frame_time[0] if _overlay_frame_time[0] else 999.0

            # Debug rendering can pause during dance/completion. The permanent
            # capture thread still publishes raw frames, so video remains live.
            if display is None or overlay_age > 0.20:
                ok, raw, _sequence, captured_at = _latest_camera_frame()
                if ok and now - captured_at < 0.50:
                    display = raw

            if display is not None:                _last_video_frame[0] = display
            elif _last_video_frame[0] is not None:
                display = _last_video_frame[0]
            else:
                if placeholder is None:
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        placeholder,
                        "Waiting for camera...",
                        (160, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (80, 80, 80),
                        2,
                    )
                display = placeholder

            ret, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 55])
            if ret:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + jpeg.tobytes()
                    + b"\r\n"
                )
        except Exception as e:
            print(f"[VideoStream] Error: {e}")
        time.sleep(0.033)


def start_navigation():
    global _navigation_thread, _navigation_stop
    try:
        import tasks.project.packages.agent as agent
    except Exception as e:
        print(f"[Navigation] Failed to import agent: {e}", flush=True)
        return False

    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running", flush=True)
        return True

    _start = current_node
    _goal = goal_node
    _heading = start_direction
    _navigation_stop = threading.Event()
    run_stop = _navigation_stop
    # Navigation consumes buffered frames; only the capture thread touches the
    # physical GStreamer camera.
    camera_view = _LatestFrameCamera(camera)
    import servers.project.real_server as _self
    _self.current_node = _start
    _self.goal_node = _goal
    _self.start_direction = _heading
    print(
        f"[Navigation] Starting - current_node={_start} goal_node={_goal} "
        f"heading={_heading} (self id={id(_self)})",
        flush=True,
    )

    def _run():
        try:
            agent.main(camera_view, wheels, leds, run_stop, _self)
        except Exception as e:
            import traceback

            print(f"[Navigation] CRASHED: {e}", flush=True)
            traceback.print_exc()

    _navigation_thread = threading.Thread(
        target=_run, daemon=True, name="NavigationThread"
    )
    _navigation_thread.start()
    return True


def stop_navigation():
    global _navigation_thread

    if not _navigation_thread or not _navigation_thread.is_alive():
        print("[Navigation] Not running")
        _navigation_thread = None
        return True

    print("[Navigation] Stopping navigation loop...")
    _navigation_stop.set()
    _navigation_thread.join(timeout=3.0)
    if _navigation_thread.is_alive():
        print("[Navigation] Stop timed out; camera ownership remains with navigation")
        return False

    _navigation_thread = None
    print("[Navigation] Navigation stopped")
    return True


@app.route("/config/<path:filename>")
def serve_config(filename):
    return send_from_directory(os.path.join(project_root, "config"), filename)


@app.route("/")
def index():
    return HTML_TEMPLATE(
        title="Navigation — Project",
        subtitle="Real Duckiebot",
    )


@app.route("/video")
def video():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def _detection_status():
    try:
        import tasks.project.packages.agent as _ag

        det = _ag.agent.detector
    except Exception as e:
        return {
            "model_loaded": False,
            "load_error": f"Agent import failed: {e}",
            "trt_building": False,
        }
    if det is None:
        return {
            "model_loaded": False,
            "load_error": getattr(_ag.agent, "detector_error", None) or "Detector not initialized",
            "trt_building": False,
        }
    return {
        "model_loaded": det.model_loaded,
        "load_error": det.load_error,
        "trt_building": getattr(det, "trt_building", False),
        "trt_build_elapsed": getattr(det, "trt_build_elapsed", 0),
        "detection_backend": getattr(det, "_backend", None),
    }


@app.route("/status")
def status():
    return jsonify(
        {
            "current_node": current_node,
            "goal_node": goal_node,
            "left_speed": round(current_speeds["left"], 2),
            "right_speed": round(current_speeds["right"], 2),
            "mode": "manual" if _manual_mode else "navigation",
            **_detection_status(),
        }
    )


@app.route("/keys", methods=["POST"])
def update_keys():
    global keys_pressed, _keys_last_update
    data = request.json
    with keys_lock:
        keys_pressed = {
            "up": bool(data.get("up", False)),
            "down": bool(data.get("down", False)),
            "left": bool(data.get("left", False)),
            "right": bool(data.get("right", False)),
        }
    _keys_last_update = time.time()

    up = keys_pressed["up"]
    down = keys_pressed["down"]
    left = keys_pressed["left"]
    right = keys_pressed["right"]

    if up and not down:
        l, r = 0.4, 0.4
    elif down and not up:
        l, r = -0.4, -0.4
    elif left and not right:
        l, r = -0.3, 0.3
    elif right and not left:
        l, r = 0.3, -0.3
    else:
        l, r = 0.0, 0.0

    current_speeds["left"] = l
    current_speeds["right"] = r

    if _manual_mode and wheels:
        wheels.set_wheels_speed(l, r)

    return jsonify({"status": "ok", "left": l, "right": r})


@app.route("/set_mode", methods=["POST"])
def set_mode():
    global _manual_mode
    requested_manual = bool(request.json.get("manual", False))

    with _mode_lock:
        if requested_manual:
            print("[Mode] Manual Drive - stopping navigation")
            # Keep manual camera reads disabled until navigation releases it.
            if not stop_navigation():
                return jsonify({
                    "status": "busy",
                    "manual": False,
                    "error": "Navigation did not release the camera",
                }), 409

            _manual_mode = True
            try:
                import tasks.project.packages.agent as agent_mod
                agent_mod.debug_frame = None
            except Exception:
                pass
            with _overlay_lock:
                _overlay_frame[0] = None
                _overlay_frame_time[0] = 0.0
            if wheels:
                wheels.set_wheels_speed(0.0, 0.0)
        else:
            print("[Mode] Autonomous Navigation - starting agent")
            # Disable manual reads before navigation requests its first frame.
            _manual_mode = False
            if not start_navigation():
                _manual_mode = True
                return jsonify({
                    "status": "error",
                    "manual": True,
                    "error": "Navigation failed to start",
                }), 500

    return jsonify({"status": "ok", "manual": _manual_mode})


@app.route("/nodes_coords")
def nodes_coords():
    from tasks.project.packages.road_map import road_map

    nodes = [
        {"id": nid, "x": ndata["x"], "y": ndata["y"]}
        for nid, ndata in road_map.nodes.items()
    ]
    return jsonify({"nodes": nodes})


@app.route("/set_start", methods=["POST"])
def set_start():
    from config.config_provider import config

    global current_node, start_direction
    current_node = int(request.json["node"])
    start_direction = request.json.get("direction", "N")

    # Save to config
    config.set_navigation(start_node=current_node, start_direction=start_direction)
    config.save()

    print(f"[Start] Intersection {current_node} direction={start_direction}")
    return jsonify({"status": "ok", "node": current_node, "direction": start_direction})


@app.route("/get_start")
def get_start():
    return jsonify({"node": current_node, "direction": start_direction})


@app.route("/get_hsv")
def get_hsv():
    from tasks.visual_lane_servoing.packages.visual_servoing_activity import get_hsv_bounds

    return jsonify(get_hsv_bounds())


@app.route("/update_hsv", methods=["POST"])
def update_hsv():
    from tasks.visual_lane_servoing.packages import visual_servoing_activity as lane_detector

    data = request.json or {}
    current = lane_detector.get_hsv_bounds()
    current.update({k: int(v) for k, v in data.items() if k in current})
    lane_detector.set_hsv_bounds(
        [current["yellow_lower_h"], current["yellow_lower_s"], current["yellow_lower_v"]],
        [current["yellow_upper_h"], current["yellow_upper_s"], current["yellow_upper_v"]],
        [current["white_lower_h"], current["white_lower_s"], current["white_lower_v"]],
        [current["white_upper_h"], current["white_upper_s"], current["white_upper_v"]],
    )

    hsv_path = os.path.join(project_root, "config", "lane_servoing_hsv_config.yaml")
    with open(hsv_path, "w") as hsv_file:
        yaml.safe_dump(current, hsv_file, default_flow_style=False)

    return jsonify({"status": "ok", "new_values": lane_detector.get_hsv_bounds()})


@app.route("/set_goal", methods=["POST"])
def set_goal():
    from config.config_provider import config

    global goal_node
    goal_node = int(request.json["node"])
    route = dijkstra(current_node, goal_node, start_direction)

    # Save to config
    config.set_navigation(goal_node=goal_node)
    config.save()

    print("\n===================")
    print("PATH PLANNER")
    print("===================")
    print(f"Start intersection: {current_node}")
    print(f"Goal intersection: {goal_node}")
    print(f"Path: {route['path']}")
    print(f"Edges: {route['edges']}")
    print(f"Distance: {route['distance']}")
    print("===================\n")

    return jsonify(
        {
            "status": "ok",
            "node": goal_node,
            "path": route["path"],
            "distance": route["distance"] if route["path"] else None,
        }
    )


@app.route("/get_goal")
def get_goal():
    return jsonify({"node": goal_node})


@app.route("/config/bots")
def get_bot_list():
    from config.config_provider import config

    return jsonify(
        {"bots": config.get_bots(), "current": config.get_current_bot_name()}
    )


@app.route("/config/load", methods=["POST"])
def load_bot_config():
    from config.config_provider import config

    bot_name = request.json.get("bot_name")
    if not bot_name:
        return jsonify({"status": "error", "message": "bot_name required"}), 400

    try:
        config.load(bot_name)
        return jsonify(
            {
                "status": "ok",
                "bot_name": bot_name,
                "message": f"Loaded configuration: {bot_name}",
            }
        )
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/config/save", methods=["POST"])
def save_bot_config():
    from config.config_provider import config

    bot_name = request.json.get("bot_name")
    if not bot_name:
        return jsonify({"status": "error", "message": "bot_name required"}), 400

    try:
        config.save(bot_name)
        return jsonify(
            {
                "status": "ok",
                "bot_name": bot_name,
                "message": f"Saved configuration as: {bot_name}",
            }
        )
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/config/download")
def download_bot_config():
    import json
    import os

    from config.config_provider import config

    try:
        config_data = config.get_all()
        config_json = json.dumps(config_data, indent=2)
        config_path = config.config_path
        filename = os.path.basename(config_path)

        return Response(
            config_json,
            mimetype="application/json",
            headers={"Content-Disposition": f"attachment; filename={filename}"},
        )
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/maneuver", methods=["POST"])
def run_maneuver():
    data = request.json
    mtype = data.get("type", "")
    value = float(data.get("value", 0.5))

    if mtype == "dance":
        distance = float(np.clip(value, 3.0, 10.0))
        start_maneuver(dance, distance)
        return jsonify({"status": "ok", "maneuver": "dance", "distance": distance})

    return jsonify({"status": "error", "message": "Unknown maneuver"}), 400


@app.route("/get_lane_config")
def get_lane_config():
    from config.config_provider import config

    return jsonify(config.get_lane_control())


@app.route("/set_lane_config", methods=["POST"])
def set_lane_config():
    import tasks.project.packages.agent as _ag
    from config.config_provider import config

    data = request.json

    # Update config
    config.set_lane_control(
        p_gain=data.get("p_gain"),
        d_gain=data.get("d_gain"),
        base_speed=data.get("base_speed"),
    )
    config.save()

    # Apply to agent
    lf = _ag.agent.lane_follower
    lane_cfg = config.get_lane_control()
    lf.p_gain = lane_cfg["p_gain"]
    lf.d_gain = lane_cfg["d_gain"]
    lf.base_speed = lane_cfg["base_speed"]

    print(f"[LaneConfig] p={lf.p_gain} d={lf.d_gain} speed={lf.base_speed}")
    return jsonify({"status": "ok", **lane_cfg})


@app.route("/get_timing_config")
def get_timing_config():
    from config.config_provider import config

    timing = config.get_timing()
    return jsonify(
        {
            "forward_clear_time": timing.get("creep_time", 0.80),
            "exit_timeout": timing.get("exit_timeout", 4.0),
            "turn_time_forward": timing.get("forward_through", 1.0),
            "turn_time_left": timing.get("left_turn", 1.10),
            "turn_time_right": timing.get("right_turn", 0.80),
            "turn_time_turnaround": timing.get("turnaround", 3.20),
        }
    )


@app.route("/set_timing_config", methods=["POST"])
def set_timing_config():
    import tasks.project.packages.agent as _ag
    from config.config_provider import config

    data = request.json

    # Map UI keys to config keys
    updates = {}
    if "forward_clear_time" in data:
        updates["creep_time"] = float(data["forward_clear_time"])
    if "exit_timeout" in data:
        updates["exit_timeout"] = float(data["exit_timeout"])
    if "turn_time_forward" in data:
        updates["forward_through"] = float(data["turn_time_forward"])
    if "turn_time_left" in data:
        updates["left_turn"] = float(data["turn_time_left"])
    if "turn_time_right" in data:
        updates["right_turn"] = float(data["turn_time_right"])
    if "turn_time_turnaround" in data:
        updates["turnaround"] = float(data["turn_time_turnaround"])

    # Update config
    config.set_timing(**updates)
    config.save()

    # Apply to agent module
    timing = config.get_timing()
    _ag.FORWARD_CLEAR_TIME = timing["creep_time"]
    _ag.FORWARD_CLEAR_TIMEOUT = _ag.FORWARD_CLEAR_TIME + (0.5 if not _ag._IS_REAL else 1.6)
    _ag.EXIT_TIMEOUT = timing["exit_timeout"]
    _ag.TURN_TIME_FORWARD = timing["forward_through"]
    _ag.TURN_TIME_LEFT = timing["left_turn"]
    _ag.TURN_TIME_RIGHT = timing["right_turn"]
    _ag.TURN_TIME_TURNAROUND = timing["turnaround"]
    _ag.TURN_TIMES["forward"] = _ag.TURN_TIME_FORWARD
    _ag.TURN_TIMES["left"] = _ag.TURN_TIME_LEFT
    _ag.TURN_TIMES["right"] = _ag.TURN_TIME_RIGHT
    _ag.TURN_TIMES["turnaround"] = _ag.TURN_TIME_TURNAROUND

    print(
        f"[TimingConfig] creep={_ag.FORWARD_CLEAR_TIME:.2f} exit={_ag.EXIT_TIMEOUT:.1f} "
        f"fwd={_ag.TURN_TIME_FORWARD:.2f} left={_ag.TURN_TIME_LEFT:.2f} right={_ag.TURN_TIME_RIGHT:.2f} "
        f"turnaround={_ag.TURN_TIME_TURNAROUND:.2f}"
    )
    return jsonify({"status": "ok"})


@app.route("/get_turn_bias")
def get_turn_bias():
    import tasks.project.packages.agent as _ag

    return jsonify(
        {
            "turn_bias_low": _ag.TURN_BIAS_LOW,
            "turn_bias_high": _ag.TURN_BIAS_HIGH,
        }
    )


@app.route("/set_turn_bias", methods=["POST"])
def set_turn_bias():
    import tasks.project.packages.agent as _ag

    data = request.json
    if "turn_bias_low" in data:
        _ag.TURN_BIAS_LOW = float(data["turn_bias_low"])
    if "turn_bias_high" in data:
        _ag.TURN_BIAS_HIGH = float(data["turn_bias_high"])
    print(
        f"[TurnBias] low={_ag.TURN_BIAS_LOW:.2f} high={_ag.TURN_BIAS_HIGH:.2f}",
        flush=True,
    )
    return jsonify({"status": "ok"})


@app.route("/speeds")
def get_speeds():
    return jsonify(current_speeds)


@app.route("/shutdown")
def shutdown():
    shutdown_cleanup(wheels, camera, stop_event)
    return jsonify({"status": "ok"})


def main():
    global camera, wheels, leds, stop_event

    ap = argparse.ArgumentParser(description="Project Server — Real Hardware")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    suppress_http_logs()
    print("=" * 60)
    print("PROJECT SERVER — REAL HARDWARE")
    print("=" * 60)

    print("\n[1/4] Initializing LED driver...")
    try:
        leds = LEDDriver()
        leds.all_off()
        print("  LEDs: ok")
    except Exception as e:
        print(f"  LEDs: not available ({e})")
        leds = None

    print("\n[2/4] Initializing wheels driver...")
    physical_wheels = DaguWheelsDriver(
        WheelPWMConfiguration(), WheelPWMConfiguration()
    )
    wheels = _ResilientWheels(physical_wheels)
    print("  Wheels: ok")

    print("\n[3/4] Initializing camera driver...")
    import subprocess

    print("  Restarting nvargus-daemon...")
    try:
        subprocess.run(
            ["sudo", "systemctl", "restart", "nvargus-daemon"],
            timeout=10,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(3.0)
    except Exception as e:
        print(f"  nvargus-daemon restart failed: {e}")
    for _attempt in range(5):
        try:
            camera = CameraDriver()
            camera.start()
            print("  Camera: ok")
            break
        except Exception as e:
            print(f"  Camera attempt {_attempt + 1}/5 failed: {e}")
            try:
                camera.stop()
            except Exception:
                pass
            camera = None
            if _attempt < 4:
                try:
                    subprocess.run(
                        ["sudo", "systemctl", "restart", "nvargus-daemon"],
                        timeout=10,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    time.sleep(3.0)
                except Exception:
                    time.sleep(2.0)
    if camera is None:
        print("  WARNING: Camera failed to start")
    else:
        _start_camera_capture_thread()
        _start_overlay_thread()

    print("\n[4/4] Ready — use the UI to start navigation")

    def _shutdown(signum, frame):
        print("\nShutting down...")
        if leds:
            try:
                leds.all_off()
                leds.release()
            except Exception:
                pass
        shutdown_cleanup(wheels, camera, stop_event)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    web_port = find_available_port(args.port)
    print(f"\nWeb UI: http://10.251.244.26:{web_port}")
    print("Press Ctrl+C to stop\n")

    try:
        _manual_mode = True
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        print("[Mode] Starting in Manual mode")
        app.run(host="0.0.0.0", port=web_port, debug=False, threaded=True)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if leds:
            try:
                leds.all_off()
                leds.release()
            except Exception:
                pass
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == "__main__":
    sys.exit(main())
