import argparse
import os
import sys
import threading
import time

import yaml

from tasks.project.packages.optimal_path import dijkstra

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, "..", "..")
sys.path.insert(0, project_root)

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

from duckiebot.camera_driver.godot_camera_driver import (
    GodotCameraConfig,
    GodotCameraDriver,
)
from duckiebot.wheel_driver.godot_wheels_driver import GodotWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from launcher.ports import find_available_port
from servers.common import shutdown_cleanup, suppress_http_logs
from servers.templates.project import get_template as HTML_TEMPLATE
from tasks.introduction.packages import manual_drive

app = Flask(__name__)
app.static_folder = os.path.join(project_root, "static")

camera = None
wheels = None
keys_pressed = {"up": False, "down": False, "left": False, "right": False}
keys_lock = threading.Lock()
_keys_last_update = time.time()
current_speeds = {"left": 0.0, "right": 0.0}
stop_event = threading.Event()
student_code_works = True
maneuver_thread = None
maneuver_stop = threading.Event()
current_node = 1
start_direction = "E"  # Default robot heading (change via UI grid picker)
goal_node = 3
_manual_mode = True
_navigation_thread = None
_navigation_stop = threading.Event()

LANE_HSV_CONFIG_FILE = os.path.join(
    project_root, "config", "lane_servoing_hsv_config.yaml"
)


def _get_student_module():
    from tasks.visual_lane_servoing.packages import visual_servoing_activity

    return visual_servoing_activity


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


def control_loop():
    global keys_pressed, current_speeds, student_code_works, _keys_last_update

    print("[ControlLoop] Starting...")

    while not stop_event.is_set():
        try:
            if time.time() - _keys_last_update > 0.5:
                with keys_lock:
                    keys_pressed = {
                        "up": False,
                        "down": False,
                        "left": False,
                        "right": False,
                    }

            with keys_lock:
                keys_copy = keys_pressed.copy()

            try:
                left, right = manual_drive.get_motor_speeds(keys_copy)
                student_code_works = True
            except Exception as e:
                print(f"[ControlLoop] Student code error: {e}")
                left, right = 0.0, 0.0
                student_code_works = False

            current_speeds["left"] = left
            current_speeds["right"] = right

            if _manual_mode and wheels:
                wheels.set_wheels_speed(left, right)

            time.sleep(0.05)

        except Exception as e:
            print(f"[ControlLoop] Error: {e}")
            time.sleep(0.1)

    print("[ControlLoop] Stopped")


def start_navigation():
    """Start the autonomous navigation thread."""
    global _navigation_thread, _navigation_stop
    import tasks.project.packages.agent as agent

    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running")
        return

    # Capture values NOW before any other thread changes them
    _start = current_node
    _goal = goal_node
    _heading = start_direction

    print(
        f"[Navigation] Starting navigation loop — current_node={_start} goal_node={_goal} heading={_heading}"
    )
    _navigation_stop.clear()
    import servers.project.virtual_server as _self

    # Set them explicitly so agent reads correct values
    _self.current_node = _start
    _self.goal_node = _goal
    _self.start_direction = _heading

    _navigation_thread = threading.Thread(
        target=agent.main,
        args=(camera, wheels, None, _navigation_stop, _self),
        daemon=True,
        name="NavigationThread",
    )
    _navigation_thread.start()


def stop_navigation():
    """Stop the autonomous navigation thread."""
    global _navigation_thread, _navigation_stop

    if not _navigation_thread or not _navigation_thread.is_alive():
        print("[Navigation] Not running")
        return

    print("[Navigation] Stopping navigation loop...")
    _navigation_stop.set()
    _navigation_thread.join(timeout=2.0)
    if _navigation_thread.is_alive():
        print("[Navigation] Warning: Navigation thread still alive after timeout")
    else:
        print("[Navigation] Navigation stopped")


def dance(duration_sec, stop_ev):
    print(f"[Dance] Starting for {duration_sec:.1f}s")
    duration = float(np.clip(duration_sec, 0.5, 10.0))

    if wheels:
        wheels.set_wheels_speed(0.8, -0.8)
        time.sleep(0.6)
        wheels.set_wheels_speed(0.8, 0.8)
        time.sleep(1.0)
        wheels.set_wheels_speed(0.0, 0.0)
        time.sleep(0.1)

    end_time = time.time() + duration
    step = 0

    while not stop_ev.is_set() and time.time() < end_time:
        if step % 2 == 0:
            left, right = 0.8, -0.8
        else:
            left, right = -0.8, 0.8

        if wheels:
            wheels.set_wheels_speed(left, right)

        time.sleep(0.1)
        step += 1

    if wheels:
        wheels.set_wheels_speed(0.0, 0.0)
    print("[Dance] Done")


def generate_frames():
    """
    MJPEG generator for /video.
    """
    while True:
        try:
            display = None
            if camera is not None:
                ok, raw_rgb = camera.read_rgb()
                if ok and raw_rgb is not None:
                    display = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)

            if display is None:
                placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder,
                    "Waiting for Godot...",
                    (200, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (100, 100, 100),
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


@app.route("/config/<path:filename>")
def serve_config(filename):
    return send_from_directory(os.path.join(project_root, "config"), filename)


@app.route("/")
def index():
    return HTML_TEMPLATE(
        title="Navigation — Project",
        subtitle="Duckiebot navigation task",
    )


@app.route("/get_hsv")
def get_hsv():
    return jsonify(_get_student_module().get_hsv_bounds())


@app.route("/update_hsv", methods=["POST"])
def update_hsv():
    data = request.json
    mod = _get_student_module()
    current = mod.get_hsv_bounds()
    current.update({k: int(v) for k, v in data.items()})
    mod.set_hsv_bounds(
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
    try:
        with open(LANE_HSV_CONFIG_FILE, "w") as f:
            yaml.dump(current, f, default_flow_style=False)
    except Exception as e:
        print(f"[Project] Could not save HSV config: {e}")
    return jsonify({"status": "ok"})


@app.route("/video")
def video():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
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
    return jsonify(
        {
            "status": "ok",
            "left": current_speeds["left"],
            "right": current_speeds["right"],
        }
    )


@app.route("/speeds")
def get_speeds():
    return jsonify(current_speeds)


@app.route("/set_mode", methods=["POST"])
def set_mode():
    global _manual_mode
    _manual_mode = bool(request.json.get("manual", False))
    if not _manual_mode and wheels:
        wheels.set_wheels_speed(0.0, 0.0)

    if _manual_mode:
        print(f"[Mode] Manual Drive - stopping navigation")
        stop_navigation()
    else:
        print(f"[Mode] Autonomous Navigation - starting agent")
        start_navigation()

    return jsonify({"status": "ok", "manual": _manual_mode})











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


@app.route("/route")
def route():
    result = dijkstra(current_node, goal_node, start_direction)
    if not result["path"]:
        result["distance"] = None
    return jsonify(result)


@app.route("/get_goal")
def get_goal():
    return jsonify({"node": goal_node})


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


def main():
    global camera, wheels, stop_event

    ap = argparse.ArgumentParser(description="Navigation Project Server")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--frame-port", type=int, default=5001)
    ap.add_argument("--wheel-port", type=int, default=5002)
    ap.add_argument("--godot-host", type=str, default="localhost")
    args = ap.parse_args()

    suppress_http_logs()
    print("=" * 60)
    print("NAVIGATION PROJECT (SIMULATION)")
    print("=" * 60)

    print("\n[1/2] Initializing wheels driver...")
    left_cfg = WheelPWMConfiguration()
    right_cfg = WheelPWMConfiguration()
    wheels = GodotWheelsDriver(
        left_cfg,
        right_cfg,
        godot_host=args.godot_host,
        godot_port=args.wheel_port,
    )
    wheels.trim = 0
    print(f"  Wheels: {args.godot_host}:{args.wheel_port}")

    print("\n[2/2] Initializing camera driver...")
    camera_cfg = GodotCameraConfig(host="0.0.0.0", port=args.frame_port)
    camera = GodotCameraDriver(godot_config=camera_cfg)
    camera.start()
    print(f"  Camera: connected!")

    stop_event.clear()
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    web_port = find_available_port(args.port)
    if web_port != args.port:
        print(f"  Port {args.port} busy, using {web_port}")

    print("\n" + "=" * 60)
    print(f"Web Interface: http://localhost:{web_port}")
    print("=" * 60 + "\n")

    try:
        _manual_mode = True
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        print("[Mode] Starting in Manual mode")
        app.run(host="127.0.0.1", port=web_port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == "__main__":
    sys.exit(main())
