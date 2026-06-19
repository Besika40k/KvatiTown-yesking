import os
import threading
import time
from collections import deque

import cv2
import numpy as np

from tasks.project.packages.optimal_path import apply_maneuver, dijkstra
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent

try:
    from tasks.object_detection.packages.agent import (
        CLASS_COLORS,
        CLASS_NAMES,
        ObjectDetectionAgent,
    )
except Exception as _e:
    ObjectDetectionAgent = None
    CLASS_NAMES, CLASS_COLORS = {}, {}
    print(f"[Agent] Object detection unavailable: {_e}", flush=True)

server = None
debug_frame = None
agent = None

# ============================================================================
# TUNING CONSTANTS
# Simulation uses short times (Godot physics runs faster).
# Real robot uses longer times to account for actual motor response.
# Set DUCKIEBOT_REAL=1 environment variable when running on real hardware,
# or it auto-detects by checking if Godot is available.
# ============================================================================

try:
    import godot

    _IS_REAL = False
except ImportError:
    _IS_REAL = True

# Override with environment variable if set
if os.environ.get("DUCKIEBOT_REAL", "") == "1":
    _IS_REAL = True
elif os.environ.get("DUCKIEBOT_SIM", "") == "1":
    _IS_REAL = False

print(f"[Agent] Running on: {'REAL ROBOT' if _IS_REAL else 'SIMULATION'}", flush=True)

# ── Speeds ────────────────────────────────────────────────────────────────────
MOTOR_BIAS = 0 if _IS_REAL else 0.0
# Per-wheel multipliers applied to TURN_SPEED during an intersection turn.
# Real robot pivots in place: inner wheel reverses (-1), outer drives forward (+1),
# reproducing the known-good (-TURN_SPEED, +TURN_SPEED) pivot turn. (Both were 0,
# which made every turn 0 speed — the robot stopped instead of turning.)
# Sim uses a forward arc (both wheels forward, different speeds).
TURN_BIAS_LOW = 0.1 if _IS_REAL else 0.1
TURN_BIAS_HIGH = 1.8 if _IS_REAL else 1.8
# Speed while slowly driving over the red stop line before turning
CREEP_SPEED = 0.06 if not _IS_REAL else 0.3

# Speed when driving forward after a turn, searching for lane markings
EXIT_SPEED = 0.20 if not _IS_REAL else 0.3

# Speed of each wheel during a left/right rotation at an intersection
TURN_SPEED  = 0.20  if not _IS_REAL else 0.35

# ── Timings ───────────────────────────────────────────────────────────────────

# How long (seconds) to creep forward over the red line before starting the turn
FORWARD_CLEAR_TIME = 0.55 if not _IS_REAL else 1.15

# Maximum seconds to drive forward after a turn while searching for lane lines.
# If lane is found earlier (300px detected), exits immediately.
EXIT_TIMEOUT = 4.0 if not _IS_REAL else 4.0

# Seconds to drive straight forward through a forward intersection (no turn)
TURN_TIME_FORWARD = 2 if not _IS_REAL else 1.4

# Seconds to rotate left at an intersection
TURN_TIME_LEFT    = 0.04  if not _IS_REAL else 0.5

# Seconds to rotate right at an intersection
TURN_TIME_RIGHT   = 0.15  if not _IS_REAL else 0.5

# Seconds to rotate for a U-turn (turnaround). Rotates the same direction as a
# left turn, just held longer so the robot swings ~180 instead of ~90.
TURN_TIME_TURNAROUND = 0.08 if not _IS_REAL else 1.85

TURN_TIMES = {
    "forward": TURN_TIME_FORWARD,
    "left": TURN_TIME_LEFT,
    "right": TURN_TIME_RIGHT,
    "turnaround": TURN_TIME_TURNAROUND,
}

# ── Detection ─────────────────────────────────────────────────────────────────
RED_WINDOW_SIZE = 12
RED_VOTE_THRESH = 0.65
RED_ARM_FRAMES = 18  # frames to drive before red line detection is armed
RED_REARM_FRAMES = 20  # frames to ignore red lines after finishing a crossing
# (prevents triggering on the same intersection's other lines)

# ── Object detection ──────────────────────────────────────────────────────────
# Classes that make the robot stop: 0 = duckie, 1 = truck (other robots).
# Signs (2) are detected and drawn in the debug view but never block driving.
OBSTACLE_CLASSES = (0, 1)

# Ignore detections with a bbox smaller than this (px) — too far away to matter
OBSTACLE_MIN_AREA = 2500

# Bottom edge of the bbox must reach below this fraction of the frame height,
# otherwise the object is far ahead (or not on the road) and we keep driving
OBSTACLE_ZONE_Y = 0.45

# Horizontal band (fractions of frame width) the bbox centre must fall inside —
# objects outside it are in the oncoming lane / off the road
OBSTACLE_ZONE_X = (0.15, 0.85)

# Consecutive frames with / without an obstacle before stopping / resuming
OBSTACLE_STOP_FRAMES = 2
OBSTACLE_CLEAR_FRAMES = 8

# ============================================================================
# RED LINE DETECTION
# ============================================================================


def detect_red_line(image):
    if image is None or len(image.shape) != 3:
        return False, None
    if image.dtype != np.uint8:
        image = (
            (np.clip(image, 0, 1) * 255).astype(np.uint8)
            if image.max() <= 1.0
            else np.clip(image, 0, 255).astype(np.uint8)
        )

    h, w = image.shape[:2]
    roi_top = int(h * 0.55)
    roi = image[roi_top:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    roi_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([165, 100, 100]), np.array([180, 255, 255])),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_top:, :] = roi_mask

    if int(np.count_nonzero(roi_mask)) < 150:
        return False, mask

    cols = np.where(roi_mask > 0)[1]
    rows = np.where(roi_mask > 0)[0]
    if len(cols) == 0:
        return False, mask

    span_x = int(cols.max() - cols.min()) + 1
    span_y = int(rows.max() - rows.min()) + 1
    aspect = span_x / max(span_y, 1)

    if aspect < 2.5 or span_x < int(w * 0.15):
        return False, mask

    return True, mask


# ============================================================================
# DEBUG FRAME
# ============================================================================


def draw_detections(frame_bgr, detections):
    if not detections:
        return frame_bgr
    out = frame_bgr.copy()
    for (x1, y1, x2, y2), score, cls_id in detections:
        color = CLASS_COLORS.get(cls_id, (255, 255, 255))
        name = CLASS_NAMES.get(cls_id, str(cls_id))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            out,
            f"{name} {score:.2f}",
            (x1, max(14, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
    return out


def build_debug_frame(raw_bgr, mask_yellow, mask_white, mask_red, state, sub, error):
    if raw_bgr is None:
        return None
    h, w = raw_bgr.shape[:2]
    panel_w, panel_h = w // 2, h // 3
    raw = raw_bgr.copy()
    label = state.upper() + (f"/{sub}" if sub else "")
    cv2.putText(raw, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    try:
        err_val = float(error) if not isinstance(error, tuple) else 0.0
    except Exception:
        err_val = 0.0
    cv2.putText(
        raw,
        f"err:{err_val:+.3f}",
        (8, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        1,
    )

    def _make_panel(mask, tint, label):
        panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        if mask is not None:
            m = mask if mask.dtype == np.uint8 else (mask * 255).astype(np.uint8)
            r = cv2.resize(m, (panel_w, panel_h), interpolation=cv2.INTER_NEAREST)
            panel[r > 0] = tint
            cv2.putText(
                panel,
                f"{label} ({int(np.count_nonzero(m))}px)",
                (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
            )
        else:
            cv2.putText(
                panel,
                f"{label} (no mask)",
                (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (80, 80, 80),
                1,
            )
        return panel

    right_col = cv2.resize(
        np.vstack(
            [
                _make_panel(mask_yellow, [0, 220, 220], "YELLOW LANE"),
                _make_panel(mask_white, [0, 220, 0], "WHITE LANE"),
                _make_panel(mask_red, [0, 0, 220], "RED LINE"),
            ]
        ),
        (panel_w, h),
    )
    return np.hstack([raw, right_col])


# ============================================================================
# HEADING + DIRECTION
# ============================================================================


def get_direction_from_route(current_node, route):
    """Return the pre-computed maneuver for the current node from the route."""
    if not route:
        return "forward"
    path = route.get("path", [])
    directions = route.get("directions", [])
    try:
        idx = path.index(current_node)
    except ValueError:
        return "forward"
    if idx >= len(directions):
        return "forward"
    return directions[idx]


# ============================================================================
# INTERSECTION FSM
# ============================================================================


class IntersectionFSM:
    def __init__(self):
        self._phase = "done"
        self._direction = "forward"
        self._phase_end = 0.0

    def reset(self):
        self._phase = "done"
        self._direction = "forward"
        self._phase_end = 0.0

    @property
    def running(self):
        return self._phase != "done"

    def start(self, direction):
        self._direction = direction
        self._enter_phase("clear")
        print(f"[Intersection] Starting — direction='{direction}'", flush=True)

    def _enter_phase(self, phase):
        self._phase = phase
        now = time.time()
        if phase == "clear":
            self._phase_end = now + FORWARD_CLEAR_TIME
        elif phase == "turn":
            if self._direction == "forward":
                self._enter_phase("exit")
                return
            self._phase_end = now + TURN_TIMES[self._direction]
        elif phase == "exit":
            self._phase_end = now + EXIT_TIMEOUT
        elif phase == "done":
            self._phase_end = 0.0

    def update(self, wheels, frame_bgr=None):
        if self._phase == "done":
            return False
        now = time.time()
        finished = now >= self._phase_end

        if self._phase == "clear":
            wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)
            if finished:
                self._enter_phase("turn")

        elif self._phase == "turn":
            if self._direction == "forward":
                wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)
            elif self._direction == "left":
                wheels.set_wheels_speed(TURN_SPEED*TURN_BIAS_LOW, TURN_SPEED*TURN_BIAS_HIGH)
            elif self._direction == "turnaround":
                wheels.set_wheels_speed(TURN_SPEED*TURN_BIAS_LOW, TURN_SPEED*TURN_BIAS_HIGH)
            else:
                wheels.set_wheels_speed(TURN_SPEED*TURN_BIAS_HIGH, TURN_SPEED*TURN_BIAS_LOW)
            if finished:
                self._enter_phase("exit")

        elif self._phase == "exit":
            wheels.set_wheels_speed(EXIT_SPEED, EXIT_SPEED)
            lane_found = False
            if frame_bgr is not None:
                try:
                    from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
                        detect_lane_markings,
                    )

                    mask_y, mask_w = detect_lane_markings(frame_bgr)
                    px = int(np.count_nonzero(mask_y)) + int(np.count_nonzero(mask_w))
                    if px >= 300:
                        print(
                            f"[Intersection] Lane found ({px}px) — resuming", flush=True
                        )
                        lane_found = True
                except Exception:
                    pass
            if lane_found or finished:
                self._enter_phase("done")
                return False

        return True


# ============================================================================
# NAVIGATION AGENT
# ============================================================================


class NavigationAgent:
    def __init__(self, start_direction="E"):
        self.lane_follower = LaneServoingAgent()
        self.lane_follower._YELLOW_TARGET = 0.30
        self.lane_follower._WHITE_TARGET = 0.72
        self.intersection_fsm = IntersectionFSM()
        self.state = "driving"
        self.current_route = None
        self._red_window = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames = 0
        self._route_initialized = False

        # Object detection runs in a background thread so inference never
        # blocks the control loop; the loop just reads the latest results.
        self.detector = None
        self._det_lock = threading.Lock()
        self._det_frame = None
        self._detections = []
        self._obstacle_streak = 0
        self._clear_streak = 0
        self._obstacle_stopped = False
        self._led_mode = None
        # Detector is lazily initialized on first use (see _ensure_detector)
        # to avoid starting GPU-heavy TensorRT compilation at import time.

        self._current_heading = start_direction

    def reset(self, start_direction="E"):
        print("[Agent] Resetting", flush=True)
        self.lane_follower._prev_error = 0.0
        self.lane_follower._filtered_error = 0.0
        self.intersection_fsm.reset()
        self.state = "driving"
        self.current_route = None
        self._red_window = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames = 0
        self._route_initialized = False
        self._obstacle_streak = 0
        self._clear_streak = 0
        self._obstacle_stopped = False
        self._led_mode = None
        with self._det_lock:
            self._det_frame = None
            self._detections = []
        self._current_heading = start_direction

    def _ensure_detector(self):
        if self.detector is None and ObjectDetectionAgent is not None:
            try:
                self.detector = ObjectDetectionAgent()
                threading.Thread(target=self._detection_worker, daemon=True).start()
            except Exception as e:
                print(f"[Agent] Object detection init failed: {e}", flush=True)

    def _detection_worker(self):
        while True:
            with self._det_lock:
                frame = self._det_frame
                self._det_frame = None
            if frame is None:
                time.sleep(0.01)
                continue
            try:
                dets = self.detector.detect(frame)
            except Exception as e:
                print(f"[Agent] Detection error: {e}", flush=True)
                dets = None
            # None means the frame was skipped (NUMBER_FRAMES_SKIPPED) —
            # keep the previous results in that case
            if dets is not None:
                with self._det_lock:
                    self._detections = dets

    @staticmethod
    def _is_obstacle(det, w, h):
        (x1, y1, x2, y2), _score, cls_id = det
        if cls_id not in OBSTACLE_CLASSES:
            return False
        if (x2 - x1) * (y2 - y1) < OBSTACLE_MIN_AREA:
            return False
        if y2 < h * OBSTACLE_ZONE_Y:
            return False
        cx = (x1 + x2) / 2.0
        return w * OBSTACLE_ZONE_X[0] <= cx <= w * OBSTACLE_ZONE_X[1]

    def _apply_leds(self, leds, mode):
        # LED indices: 0=front-left, 2=front-right, 3=back-right, 4=back-left
        if self._led_mode == mode:
            return
        self._led_mode = mode
        if leds is None:
            return
        try:
            if mode == "driving":
                for led in (0, 2, 3, 4):
                    leds.set_rgb(led, [1.0, 1.0, 1.0])
            elif mode == "red_stop":
                leds.set_rgb(0, [1.0, 1.0, 1.0])  # front-left white
                leds.set_rgb(2, [1.0, 1.0, 1.0])  # front-right white
                leds.set_rgb(3, [1.0, 0.0, 0.0])  # back-right red
                leds.set_rgb(4, [1.0, 0.0, 0.0])  # back-left red
            elif mode == "obstacle":
                for led in (0, 2, 3, 4):
                    leds.set_rgb(led, [1.0, 0.0, 0.0])
            elif mode == "turn_right":
                leds.set_rgb(2, [1.0, 0.6, 0.0])  # front-right yellow
                leds.set_rgb(3, [1.0, 0.6, 0.0])  # back-right yellow
                leds.set_rgb(0, [0.0, 0.0, 0.0])  # front-left off
                leds.set_rgb(4, [0.0, 0.0, 0.0])  # back-left off
            elif mode == "turn_left":
                leds.set_rgb(0, [1.0, 0.6, 0.0])  # front-left yellow
                leds.set_rgb(4, [1.0, 0.6, 0.0])  # back-left yellow
                leds.set_rgb(2, [0.0, 0.0, 0.0])  # front-right off
                leds.set_rgb(3, [0.0, 0.0, 0.0])  # back-right off
        except Exception:
            pass

    def _update_obstacle(self, detections, w, h, leds):
        """Returns True while the robot should stay stopped for an obstacle."""
        blocking = [d for d in detections if self._is_obstacle(d, w, h)]
        if blocking:
            self._obstacle_streak += 1
            self._clear_streak = 0
        else:
            self._clear_streak += 1
            self._obstacle_streak = 0

        if self._obstacle_stopped:
            if self._clear_streak >= OBSTACLE_CLEAR_FRAMES:
                self._obstacle_stopped = False
                print("[Agent] Obstacle cleared — resuming", flush=True)
                self._apply_leds(leds, "driving")
        elif self._obstacle_streak >= OBSTACLE_STOP_FRAMES:
            self._obstacle_stopped = True
            labels = sorted({CLASS_NAMES.get(d[2], str(d[2])) for d in blocking})
            print(
                f"[Agent] Obstacle ahead ({', '.join(labels)}) — stopping", flush=True
            )
            self._apply_leds(leds, "obstacle")
        return self._obstacle_stopped

    def _transition(self, new_state):
        print(f"[Agent] {self.state} -> {new_state}", flush=True)
        self.state = new_state

    def _advance_node(self):
        if self.current_route is None:
            return
        path = self.current_route.get("path", [])
        current = server.current_node
        try:
            idx = path.index(current)
        except ValueError:
            return
        if idx + 1 < len(path):
            server.current_node = path[idx + 1]
            print(
                f"[Agent] Node advanced: {current} -> {server.current_node}", flush=True
            )

    def _red_vote(self, detected):
        self._red_window.append(1 if detected else 0)
        if len(self._red_window) < RED_WINDOW_SIZE:
            return False
        return (sum(self._red_window) / RED_WINDOW_SIZE) >= RED_VOTE_THRESH

    def update(self, frame_bgr, wheels, leds, current_node, goal_node):
        global debug_frame
        red_mask = None
        fsm_phase = None
        detections = []

        if self.state == "completed":
            wheels.set_wheels_speed(0.0, 0.0)
            self._transition("celebrating")
            return True

        if self.state == "celebrating":
            wheels.set_wheels_speed(0.0, 0.0)
            return False

        if self.state == "crossing":
            fsm_phase = self.intersection_fsm._phase
            fsm_dir = self.intersection_fsm._direction
            if fsm_phase == "turn":
                if fsm_dir in ("left", "turnaround"):
                    self._apply_leds(leds, "turn_left")
                elif fsm_dir == "right":
                    self._apply_leds(leds, "turn_right")
                else:
                    self._apply_leds(leds, "driving")
            else:
                self._apply_leds(leds, "driving")
            still_running = self.intersection_fsm.update(wheels, frame_bgr)
            if not still_running:
                self._current_heading = apply_maneuver(
                    self._current_heading, self.intersection_fsm._direction
                )
                print(f"[Heading] now '{self._current_heading}'", flush=True)
                self._advance_node()

                # NEW: check arrival immediately after advancing, before resuming driving
                if server.current_node == goal_node:
                    print(f"[Agent] Reached goal node {goal_node} — completing", flush=True)
                    self._transition("completed")
                    if frame_bgr is not None:
                        debug_frame = build_debug_frame(
                            frame_bgr, None, None, None, self.state, fsm_phase, 0.0
                        )
                    return True

                self.current_route = None
                self._route_initialized = False
                self._driving_frames = -RED_REARM_FRAMES
                self._red_window.clear()
                self.lane_follower._prev_error = 0.0
                self.lane_follower._filtered_error = 0.0
                self._transition("driving")
            if frame_bgr is not None:
                debug_frame = build_debug_frame(
                    frame_bgr, None, None, None, self.state, fsm_phase, 0.0
                )
            return True

        if self.state == "driving":
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            self._ensure_detector()
            if self.detector is not None:
                with self._det_lock:
                    self._det_frame = frame_rgb
                    detections = list(self._detections)
                h, w = frame_bgr.shape[:2]
                if self._update_obstacle(detections, w, h, leds):
                    wheels.set_wheels_speed(0.0, 0.0)
                    debug_frame = build_debug_frame(
                        draw_detections(frame_bgr, detections),
                        None,
                        None,
                        None,
                        self.state,
                        "obstacle",
                        0.0,
                    )
                    return True

            self._apply_leds(leds, "driving")
            left, right = self.lane_follower.compute_commands(frame_rgb)

            di = self.lane_follower.last_debug_info
            no_lane = (
                (di.get("total_lane_pixels", 0) < di.get("detection_threshold", 50))
                if di
                else False
            )
            if no_lane:
                left = EXIT_SPEED
                right = EXIT_SPEED

            self._driving_frames += 1
            armed = self._driving_frames >= RED_ARM_FRAMES

            if not armed:
                wheels.set_wheels_speed(left, right + MOTOR_BIAS)
            else:
                red_detected, red_mask = detect_red_line(frame_bgr)
                confirmed = self._red_vote(red_detected)

                vote_fraction = sum(self._red_window) / max(len(self._red_window), 1)
                if vote_fraction > 0.3:
                    speed_scale = max(0.0, 1.0 - vote_fraction)
                    wheels.set_wheels_speed(left * speed_scale, right * speed_scale)
                else:
                    wheels.set_wheels_speed(left, right + MOTOR_BIAS)

                if confirmed:
                    self._apply_leds(leds, "red_stop")
                    wheels.set_wheels_speed(0.0, 0.0)
                    self._red_window.clear()
                    self._driving_frames = 0

                    if not self._route_initialized:
                        print(
                            f"[Agent] First red line at node {server.current_node}",
                            flush=True,
                        )
                        self.current_route = dijkstra(
                            server.current_node, goal_node, self._current_heading
                        )
                        self.current_route["start"] = server.current_node
                        print(
                            f"[Agent] Route: {self.current_route['path']}", flush=True
                        )
                        print(
                            f"[Agent] Edges: {self.current_route['edges']}", flush=True
                        )
                        self._route_initialized = True

                    if server.current_node == goal_node:
                        self._transition("completed")
                        return False

                    print(
                        f"[Agent] Red line confirmed — crossing | node={server.current_node} "
                        f"route={self.current_route.get('path') if self.current_route else None}",
                        flush=True,
                    )
                    direction = get_direction_from_route(
                        server.current_node, self.current_route
                    )
                    self.intersection_fsm.start(direction)
                    self._transition("crossing")

        if frame_bgr is not None:
            di = self.lane_follower.last_debug_info
            debug_frame = build_debug_frame(
                raw_bgr=draw_detections(frame_bgr, detections),
                mask_yellow=di.get("yellow_mask"),
                mask_white=di.get("white_mask"),
                mask_red=red_mask,
                state=self.state,
                sub=fsm_phase,
                error=self.lane_follower._prev_error,
            )
        return True


agent = NavigationAgent()


# ============================================================================
# MAIN LOOP
# ============================================================================


def main(camera, wheels, leds, stop_event, server_module=None):
    global server, debug_frame

    if server_module is not None:
        server = server_module

    start_dir = getattr(server, "start_direction", "E")
    agent.reset(start_dir)
    debug_frame = None

    print(
        f"[Agent] Started — Start: {server.current_node}  Goal: {server.goal_node}  Heading: {agent._current_heading}",
        flush=True,
    )

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal = server.goal_node

            if hasattr(camera, "read_rgb"):
                ok, frame_rgb = camera.read_rgb()
                if not ok or frame_rgb is None:
                    time.sleep(0.02)
                    continue
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ok, frame_bgr = camera.read()
                if not ok or frame_bgr is None:
                    time.sleep(0.02)
                    continue
                if len(frame_bgr.shape) == 3 and frame_bgr.shape[2] == 4:
                    frame_bgr = frame_bgr[:, :, :3]

            should_continue = agent.update(frame_bgr, wheels, leds, start, goal)

            if not should_continue:
                print("[Agent] Route complete — dancing", flush=True)
                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(2.0)

                end_time = time.time() + 6.0
                step = 0
                dance_colors = [
                    [1.0, 0.0, 0.0],  # red
                    [0.0, 0.0, 1.0],  # blue
                    [0.0, 1.0, 0.0],  # green
                ]
                while time.time() < end_time and not stop_event.is_set():
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

                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                if leds:
                    try:
                        leds.all_off()
                    except Exception:
                        pass
                print("[Agent] Dance done", flush=True)
                break

            time.sleep(0.02)

    finally:
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            try:
                leds.all_off()  
            except Exception:
                pass
        print("[Agent] Stopped", flush=True)
