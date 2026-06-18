import argparse
import os
import signal
import sys
import threading
import time

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
keys_pressed = {"up": False, "down": False, "left": False, "right": False}
keys_lock = threading.Lock()
_keys_last_update = time.time()
current_speeds = {"left": 0.0, "right": 0.0}

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
_overlay_lock = threading.Lock()


def _overlay_loop():
    import tasks.project.packages.agent as agent_mod

    while True:
        try:
            # Navigation: use agent debug frame
            if agent_mod.debug_frame is not None:
                with _overlay_lock:
                    _overlay_frame[0] = agent_mod.debug_frame
                time.sleep(0.033)
                continue

            # Manual: read camera and run detection
            if camera is None:
                time.sleep(0.05)
                continue

            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(0.033)
                continue

            if len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]

            try:
                from tasks.project.packages.agent import build_debug_frame
                from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
                    detect_lane_markings,
                )

                mask_y_f, mask_w_f = detect_lane_markings(frame)
                display = build_debug_frame(
                    raw_bgr=frame,
                    mask_yellow=(mask_y_f * 255).astype(np.uint8),
                    mask_white=(mask_w_f * 255).astype(np.uint8),
                    mask_red=None,
                    state="manual",
                    sub=None,
                    error=0.0,
                )
            except Exception:
                display = frame

            with _overlay_lock:
                _overlay_frame[0] = display

        except Exception:
            time.sleep(0.05)


threading.Thread(target=_overlay_loop, daemon=True).start()


def generate_frames():
    placeholder = None
    while True:
        try:
            with _overlay_lock:
                display = _overlay_frame[0]

            if display is None:
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

            ret, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 70])
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
        return
    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running", flush=True)
        return
    # Capture node values NOW before any other thread changes them
    _start = current_node
    _goal = goal_node
    _heading = start_direction
    _navigation_stop.clear()
    import servers.project.real_server as _self

    # Set them explicitly so agent reads correct values
    _self.current_node = _start
    _self.goal_node = _goal
    _self.start_direction = _heading
    print(
        f"[Navigation] Starting — current_node={_start} goal_node={_goal} heading={_heading} (self id={id(_self)})",
        flush=True,
    )

    def _run():
        try:
            agent.main(camera, wheels, leds, _navigation_stop, _self)
        except Exception as e:
            import traceback

            print(f"[Navigation] CRASHED: {e}", flush=True)
            traceback.print_exc()

    _navigation_thread = threading.Thread(
        target=_run, daemon=True, name="NavigationThread"
    )
    _navigation_thread.start()


def stop_navigation():
    global _navigation_thread, _navigation_stop

    if not _navigation_thread or not _navigation_thread.is_alive():
        print("[Navigation] Not running")
        return

    print("[Navigation] Stopping navigation loop...")
    _navigation_stop.set()
    _navigation_thread.join(timeout=2.0)
    print("[Navigation] Navigation stopped")


@app.route("/config/<path:filename>")
def serve_config(filename):
    return send_from_directory(os.path.join(project_root, "config"), filename)


@app.route("/")
def index():
    base = HTML_TEMPLATE(
        title="Navigation — Project",
        subtitle="Real Duckiebot",
    )
    return base


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
            "load_error": "Detector not initialized",
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
    _manual_mode = bool(request.json.get("manual", False))
    if _manual_mode:
        print("[Mode] Manual Drive - stopping navigation")
        stop_navigation()
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
    else:
        print("[Mode] Autonomous Navigation - starting agent")
        start_navigation()
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
    global current_node, start_direction
    current_node = int(request.json["node"])
    start_direction = request.json.get("direction", "N")
    print(f"[Start] Intersection {current_node} direction={start_direction}")
    return jsonify({"status": "ok", "node": current_node, "direction": start_direction})


@app.route("/get_start")
def get_start():
    return jsonify({"node": current_node, "direction": start_direction})


@app.route("/get_hsv")
def get_hsv():
    from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
        get_hsv_bounds,
    )

    return jsonify(get_hsv_bounds())


@app.route("/update_hsv", methods=["POST"])
def update_hsv():
    from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
        get_hsv_bounds,
        set_hsv_bounds,
    )

    data = request.json
    current = get_hsv_bounds()
    current.update({k: int(v) for k, v in data.items() if k in current})
    set_hsv_bounds(
        [
            current["yellow_lower_h"],
            current["yellow_lower_s"],
            current["yellow_lower_v"],
        ],
        [
            current["yellow_upper_h"],
            current["yellow_upper_s"],
            current["yellow_upper_v"],
        ],
        [current["white_lower_h"], current["white_lower_s"], current["white_lower_v"]],
        [current["white_upper_h"], current["white_upper_s"], current["white_upper_v"]],
    )
    return jsonify({"status": "ok", "new_values": get_hsv_bounds()})


@app.route("/set_goal", methods=["POST"])
def set_goal():
    global goal_node
    goal_node = int(request.json["node"])
    route = dijkstra(current_node, goal_node, start_direction)

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
    wheels = DaguWheelsDriver(WheelPWMConfiguration(), WheelPWMConfiguration())
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
