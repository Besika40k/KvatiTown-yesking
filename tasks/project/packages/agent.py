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
latest_camera_frame = None
latest_camera_frame_time = 0.0
agent = None

# ============================================================================
# TUNING CONSTANTS
# ============================================================================

try:
    import godot
    _IS_REAL = False
except ImportError:
    _IS_REAL = True

if os.environ.get("DUCKIEBOT_REAL", "") == "1":
    _IS_REAL = True
elif os.environ.get("DUCKIEBOT_SIM", "") == "1":
    _IS_REAL = False

print(f"[Agent] Running on: {'REAL ROBOT' if _IS_REAL else 'SIMULATION'}", flush=True)

# ── Speeds ────────────────────────────────────────────────────────────────────
MOTOR_BIAS     = 0    if _IS_REAL else 0.0
TURN_BIAS_LOW  = 0.00 if _IS_REAL else 0.1
TURN_BIAS_HIGH = 2.20 if _IS_REAL else 1.8
CREEP_SPEED    = 0.06 if not _IS_REAL else 0.22
EXIT_SPEED     = 0.20 if not _IS_REAL else 0.3
TURN_SPEED     = 0.20 if not _IS_REAL else 0.26

# ── Timings ───────────────────────────────────────────────────────────────────
FORWARD_CLEAR_TIME         = 0.55 if not _IS_REAL else 1.5
CLEAR_AFTER_RED_LOST_TIME  = 0.15 if not _IS_REAL else 1.05
SIDE_RED_TURN_CUE_ARM_TIME = 0.15 if not _IS_REAL else 0.25
SIDE_RED_TURN_CUE_MIN_PX   = 80   if not _IS_REAL else 140
SIDE_RED_TURN_CUE_LEFT_X   = 0.45
FORWARD_CLEAR_TIMEOUT      = FORWARD_CLEAR_TIME + (0.5 if not _IS_REAL else 1.6)
EXIT_TIMEOUT               = 2.5  if not _IS_REAL else 2.5
TURN_TIME_FORWARD          = 2    if not _IS_REAL else 2.5

# Open-loop turn times — used as hard safety timeouts when visual servo
# hasn't finished in time, NOT as the primary turn controller.
TURN_TIME_LEFT             = 0.04 if not _IS_REAL else 1.25
TURN_TIME_RIGHT            = 0.15 if not _IS_REAL else 1.25
TURN_TIME_TURNAROUND       = 0.08 if not _IS_REAL else 3.20

TURN_PULSE_TIME  = 0.55 if _IS_REAL else 999.0
TURN_PULSE_PAUSE = 0.12 if _IS_REAL else 0.0

TURN_TIMES = {
    "forward":    TURN_TIME_FORWARD,
    "left":       TURN_TIME_LEFT,
    "right":      TURN_TIME_RIGHT,
    "turnaround": TURN_TIME_TURNAROUND,
}

# ── Visual-servo turn parameters ──────────────────────────────────────────────
# Hard timeout: if the lane follower hasn't finished in this many seconds,
# fall through to the exit phase regardless.
VISUAL_TURN_TIMEOUT = 4.0 if not _IS_REAL else 7.0
RIGHT_TURN_MIN_DURATION = 0.20 if not _IS_REAL else 0.95
RIGHT_TURN_INNER_SPEED = 0.04 if not _IS_REAL else 0.06
RIGHT_TURN_OUTER_SPEED = 0.22 if not _IS_REAL else 0.30
RIGHT_TURN_VISUAL_GAIN = 0.08 if not _IS_REAL else 0.10

# ── Detection ─────────────────────────────────────────────────────────────────
RED_WINDOW_SIZE  = 12
RED_VOTE_THRESH  = 0.65
RED_ARM_FRAMES   = 18
RED_REARM_FRAMES = 20

# ── Object detection ──────────────────────────────────────────────────────────
OBSTACLE_CLASSES      = (0, 1)
OBSTACLE_MIN_AREA     = 2500
OBSTACLE_ZONE_Y       = 0.45
OBSTACLE_ZONE_X       = (0.15, 0.85)
OBSTACLE_STOP_FRAMES  = 2
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

    h, w     = image.shape[:2]
    roi_top  = int(h * 0.55)
    roi      = image[roi_top:, :]
    hsv      = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    roi_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([165, 100, 100]), np.array([180, 255, 255])),
    )
    kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_top:, :] = roi_mask

    if int(np.count_nonzero(roi_mask)) < 150:
        return False, mask

    cols   = np.where(roi_mask > 0)[1]
    rows   = np.where(roi_mask > 0)[0]
    if len(cols) == 0:
        return False, mask

    span_x = int(cols.max() - cols.min()) + 1
    span_y = int(rows.max() - rows.min()) + 1
    aspect = span_x / max(span_y, 1)

    if aspect < 2.5 or span_x < int(w * 0.15):
        return False, mask

    return True, mask


def detect_side_red_turn_cue(image, side="left"):
    if image is None or len(image.shape) != 3:
        return False, 0, None
    if image.dtype != np.uint8:
        image = (
            (np.clip(image, 0, 1) * 255).astype(np.uint8)
            if image.max() <= 1.0
            else np.clip(image, 0, 255).astype(np.uint8)
        )

    h, w    = image.shape[:2]
    roi_top = int(h * 0.45)
    roi     = image[roi_top:, :]
    hsv     = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    roi_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([165, 100, 100]), np.array([180, 255, 255])),
    )
    kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

    if side == "left":
        x1, x2 = 0, int(w * SIDE_RED_TURN_CUE_LEFT_X)
    else:
        x1, x2 = int(w * (1.0 - SIDE_RED_TURN_CUE_LEFT_X)), w

    side_mask = roi_mask[:, x1:x2]
    px   = int(np.count_nonzero(side_mask))
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_top:, :] = roi_mask
    return px >= SIDE_RED_TURN_CUE_MIN_PX, px, mask


# ============================================================================
# DEBUG FRAME
# ============================================================================

def draw_detections(frame_bgr, detections):
    if not detections:
        return frame_bgr
    out = frame_bgr.copy()
    for (x1, y1, x2, y2), score, cls_id in detections:
        color = CLASS_COLORS.get(cls_id, (255, 255, 255))
        name  = CLASS_NAMES.get(cls_id, str(cls_id))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, f"{name} {score:.2f}", (x1, max(14, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return out


def build_debug_frame(raw_bgr, mask_yellow, mask_white, mask_red, state, sub, error):
    if raw_bgr is None:
        return None
    h, w         = raw_bgr.shape[:2]
    panel_w, panel_h = w // 2, h // 3
    raw = raw_bgr.copy()
    label = state.upper() + (f"/{sub}" if sub else "")
    cv2.putText(raw, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    try:
        err_val = float(error) if not isinstance(error, tuple) else 0.0
    except Exception:
        err_val = 0.0
    cv2.putText(raw, f"err:{err_val:+.3f}", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    def _make_panel(mask, tint, label):
        panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        if mask is not None:
            m = mask if mask.dtype == np.uint8 else (mask * 255).astype(np.uint8)
            r = cv2.resize(m, (panel_w, panel_h), interpolation=cv2.INTER_NEAREST)
            panel[r > 0] = tint
            cv2.putText(panel, f"{label} ({int(np.count_nonzero(m))}px)",
                        (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        else:
            cv2.putText(panel, f"{label} (no mask)", (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 80), 1)
        return panel

    right_col = cv2.resize(
        np.vstack([
            _make_panel(mask_yellow, [0, 220, 220], "YELLOW LANE"),
            _make_panel(mask_white,  [0, 220,   0], "WHITE LANE"),
            _make_panel(mask_red,    [0,   0, 220], "RED LINE"),
        ]),
        (panel_w, h),
    )
    return np.hstack([raw, right_col])


# ============================================================================
# HEADING + DIRECTION
# ============================================================================

def get_direction_from_route(current_node, route):
    if not route:
        return "forward"
    path       = route.get("path", [])
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
    """
    Phases
    ------
    clear  – creep forward until past the stop line (unchanged)
    turn   – execute the turn using the lane-follower's own state machine
             (left_turn_enabled=True).  Open-loop is kept as a safety fallback
             only for right turns and as a hard timeout for all turns.
    exit   – drive straight until lane markings are confidently re-acquired
    done   – idle

    Visual-servo turn design
    ------------------------
    Left / turnaround
        The LaneServoingAgent already contains a left-turn state machine
        ('none' → 'straight' → 'turning') triggered by yellow line disappearing.
        We enable it (left_turn_enabled=True), then call compute_commands() every
        frame.  The agent drives itself through the bend and signals completion by
        returning to _left_turn_state == 'none' with its cooldown set.
        We detect completion by watching that state flag directly.

    Right
        The agent has no built-in right-turn logic, so we start with the original
        open-loop spin.  Once white lane pixels reappear (≥ 2 slices), we hand
        control back to the lane follower immediately — this achieves the same
        smooth hand-off without needing extra parameters.

    Forward
        Skips the turn phase entirely (goes straight to exit), same as before.

    In all cases a VISUAL_TURN_TIMEOUT hard limit prevents getting stuck.
    """

    def __init__(self):
        self._phase        = "done"
        self._direction    = "forward"
        self._phase_end    = 0.0
        self._phase_start  = 0.0
        self._clear_min_end = 0.0
        self._red_lost_at  = None

        # Shared reference set by NavigationAgent after construction
        self.lane_follower: LaneServoingAgent = None

    def reset(self):
        self._phase        = "done"
        self._direction    = "forward"
        self._phase_end    = 0.0
        self._phase_start  = 0.0
        self._clear_min_end = 0.0
        self._red_lost_at  = None

    @property
    def running(self):
        return self._phase != "done"

    def start(self, direction):
        self._direction = direction
        self._enter_phase("clear")
        print(
            f"[Intersection] Starting — direction='{direction}' "
            f"clear={FORWARD_CLEAR_TIME:.2f}s timeout={FORWARD_CLEAR_TIMEOUT:.2f}s",
            flush=True,
        )

    def _enter_phase(self, phase):
        self._phase       = phase
        now               = time.time()
        self._phase_start = now

        if phase == "clear":
            self._phase_end     = now + FORWARD_CLEAR_TIMEOUT
            self._clear_min_end = now + FORWARD_CLEAR_TIME
            self._red_lost_at   = None

        elif phase == "turn":
            if self._direction == "forward":
                self._enter_phase("exit")
                return

            # Safety net: if visual servo never finishes, this expires the phase.
            self._phase_end = now + VISUAL_TURN_TIMEOUT

            if self._direction in ("left", "turnaround"):
                # Arm the lane-follower's left-turn state machine.
                # It triggers itself when yellow disappears, but we prime the
                # visible-frames counter so it fires on the very next yellow loss.
                if self.lane_follower is not None:
                    self.lane_follower.left_turn_enabled        = True
                    self.lane_follower._yellow_visible_frames   = 999
                    self.lane_follower._left_turn_state         = "none"
                    self.lane_follower._left_turn_cooldown_end  = 0.0

        elif phase == "exit":
            self._phase_end = now + EXIT_TIMEOUT
            # Disable left-turn mode so normal lane following resumes
            if self.lane_follower is not None:
                self.lane_follower.left_turn_enabled = False

        elif phase == "done":
            self._phase_end = 0.0
            if self.lane_follower is not None:
                self.lane_follower.left_turn_enabled = False

    # ------------------------------------------------------------------
    # Open-loop right/turnaround wheel command (safety fallback only)
    # ------------------------------------------------------------------

    def _open_loop_spin(self, wheels):
        turning_now = True
        if _IS_REAL and self._direction != "forward":
            cycle       = TURN_PULSE_TIME + TURN_PULSE_PAUSE
            elapsed     = time.time() - self._phase_start
            turning_now = (elapsed % cycle) < TURN_PULSE_TIME

        if not turning_now:
            wheels.set_wheels_speed(0.0, 0.0)
        elif self._direction == "right":
            wheels.set_wheels_speed(TURN_SPEED * TURN_BIAS_HIGH, TURN_SPEED * TURN_BIAS_LOW)
        else:
            # turnaround open-loop (should rarely be reached given visual servo)
            wheels.set_wheels_speed(TURN_SPEED * TURN_BIAS_LOW, TURN_SPEED * TURN_BIAS_HIGH)

    # ------------------------------------------------------------------
    # Main update — called every control frame while FSM is running
    # ------------------------------------------------------------------

    def update(self, wheels, frame_bgr=None):
        if self._phase == "done":
            return False

        now      = time.time()
        finished = now >= self._phase_end  # hard timeout for the current phase

        # ── CLEAR ──────────────────────────────────────────────────────────
        if self._phase == "clear":
            wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)

            red_visible = False
            if frame_bgr is not None:
                try:
                    red_visible, _ = detect_red_line(frame_bgr)
                except Exception:
                    pass
            if not red_visible and self._red_lost_at is None:
                self._red_lost_at = now

            cue_seen   = False
            cue_reason = None
            cue_px     = 0
            cue_side   = "left" if self._direction in ("left", "turnaround") else "right"
            cue_armed  = (
                self._direction in ("left", "right", "turnaround")
                and self._red_lost_at is not None
                and now - self._red_lost_at >= SIDE_RED_TURN_CUE_ARM_TIME
            )
            if cue_armed and frame_bgr is not None:
                try:
                    cue_seen, cue_px, _ = detect_side_red_turn_cue(frame_bgr, cue_side)
                except Exception:
                    cue_seen, cue_px = False, 0
                if cue_seen:
                    cue_reason = f"side-red-{cue_side} px={cue_px}"

            clear_elapsed = now >= self._clear_min_end
            if cue_seen or clear_elapsed or finished:
                reason = cue_reason or ("timer" if clear_elapsed else "timeout")
                print(
                    f"[Intersection] Clear complete after {now - self._phase_start:.2f}s "
                    f"(target={FORWARD_CLEAR_TIME:.2f}s, reason={reason})",
                    flush=True,
                )
                self._enter_phase("turn")

        # ── TURN ───────────────────────────────────────────────────────────
        elif self._phase == "turn":
            lf = self.lane_follower

            # ── Left / turnaround: visual-servo via the agent's state machine ──
            if self._direction in ("left", "turnaround"):
                if lf is not None and frame_bgr is not None:
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    left, right = lf.compute_commands(frame_rgb)
                    wheels.set_wheels_speed(left, right)

                    # The agent's state machine returns to 'none' once it has
                    # re-acquired the white line (or timed out internally).
                    # That is our signal to advance to the exit phase.
                    min_turn_time = getattr(lf, "_left_turn_min_duration", 1.15)
                    turn_done = (lf._left_turn_state == "none"
                                 and now - self._phase_start > min_turn_time)
                    if turn_done:
                        elapsed = now - self._phase_start
                        print(
                            f"[Intersection] Visual {self._direction} turn complete "
                            f"in {elapsed:.2f}s",
                            flush=True,
                        )
                        self._enter_phase("exit")
                    elif finished:
                        print(
                            f"[Intersection] Visual turn timeout ({VISUAL_TURN_TIMEOUT:.1f}s) "
                            f"— forcing exit",
                            flush=True,
                        )
                        self._enter_phase("exit")
                else:
                    # No frame / no follower — open-loop fallback
                    self._open_loop_spin(wheels)
                    if finished:
                        self._enter_phase("exit")

            # Right: visual lane-assisted arc until white is re-acquired, then hand off.
            else:
                elapsed = now - self._phase_start
                right_min_elapsed = elapsed >= RIGHT_TURN_MIN_DURATION
                white_reacquired = False
                visual_commanded = False

                if frame_bgr is not None and lf is not None:
                    try:
                        from tasks.visual_lane_servoing.packages.agent import detect_lines_in_slices
                        from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
                            detect_lane_markings,
                        )

                        mask_y, mask_w = detect_lane_markings(frame_bgr)
                        h_f, w_f = mask_w.shape
                        yellow_xs, white_xs = detect_lines_in_slices(
                            (mask_y * 255).astype(np.uint8),
                            (mask_w * 255).astype(np.uint8),
                            h_f,
                        )
                        white_reacquired = right_min_elapsed and len(white_xs) >= 2

                        # Keep a right arc, but let the observed white line trim the angle.
                        # If white is too far left/center, keep turning harder right; if it
                        # is already near the right lane edge, soften the turn.
                        correction = 0.0
                        if white_xs:
                            white_mean = float(np.mean(white_xs))
                            target_x = w_f * 0.72
                            correction = float(np.clip((target_x - white_mean) / (w_f / 2.0), -1.0, 1.0))
                            correction *= RIGHT_TURN_VISUAL_GAIN

                        left_cmd = RIGHT_TURN_OUTER_SPEED + max(0.0, correction)
                        right_cmd = RIGHT_TURN_INNER_SPEED - min(0.0, correction)
                        wheels.set_wheels_speed(
                            float(np.clip(left_cmd, 0.0, 0.45)),
                            float(np.clip(right_cmd, 0.0, 0.25)),
                        )
                        visual_commanded = True
                    except Exception as exc:
                        print(f"[Intersection] Right visual turn fallback: {exc}", flush=True)

                if white_reacquired:
                    print(
                        f"[Intersection] Visual right turn complete after {elapsed:.2f}s",
                        flush=True,
                    )
                    self._enter_phase("exit")
                elif finished:
                    print(
                        f"[Intersection] Right turn timeout - forcing exit", flush=True
                    )
                    self._enter_phase("exit")
                elif not visual_commanded:
                    self._open_loop_spin(wheels)

        # ── EXIT ───────────────────────────────────────────────────────────
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
                        print(f"[Intersection] Lane found ({px}px) — resuming", flush=True)
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
        self.lane_follower.left_turn_enabled = False   # FSM controls this
        self.lane_follower.apriltag_stop     = False
        self.lane_follower._YELLOW_TARGET    = 0.30
        self.lane_follower._WHITE_TARGET     = 0.72

        self.intersection_fsm = IntersectionFSM()
        self.intersection_fsm.lane_follower = self.lane_follower  # shared reference

        self.state              = "driving"
        self.current_route      = None
        self._red_window        = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames    = 0
        self._route_initialized = False

        self.detector          = None
        self._det_lock         = threading.Lock()
        self._det_frame        = None
        self._detections       = []
        self._obstacle_streak  = 0
        self._clear_streak     = 0
        self._obstacle_stopped = False
        self._led_mode         = None

        if ObjectDetectionAgent is not None:
            try:
                self.detector = ObjectDetectionAgent()
                threading.Thread(target=self._detection_worker, daemon=True).start()
            except Exception as e:
                print(f"[Agent] Object detection init failed: {e}", flush=True)

        self._current_heading = start_direction

    def reset(self, start_direction="E"):
        print("[Agent] Resetting", flush=True)
        self.lane_follower._prev_error     = 0.0
        self.lane_follower._filtered_error = 0.0
        self.lane_follower.left_turn_enabled = False
        self.intersection_fsm.reset()
        self.intersection_fsm.lane_follower = self.lane_follower  # re-attach after reset
        self.state              = "driving"
        self.current_route      = None
        self._red_window        = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames    = 0
        self._route_initialized = False
        self._obstacle_streak   = 0
        self._clear_streak      = 0
        self._obstacle_stopped  = False
        self._led_mode          = None
        with self._det_lock:
            self._det_frame  = None
            self._detections = []
        self._current_heading = start_direction

    def _detection_worker(self):
        while True:
            with self._det_lock:
                frame           = self._det_frame
                self._det_frame = None
            if frame is None:
                time.sleep(0.01)
                continue
            try:
                dets = self.detector.detect(frame)
            except Exception as e:
                print(f"[Agent] Detection error: {e}", flush=True)
                dets = None
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
        front_white_leds = (0, 1, 2)
        driving_leds = (0, 1, 2, 3, 4)
        if self._led_mode == mode and mode != "driving":
            return
        self._led_mode = mode
        if leds is None:
            return
        try:
            if mode == "driving":
                for led in driving_leds:
                    leds.set_rgb(led, [1.0, 1.0, 1.0])
            elif mode == "red_stop":
                for led in front_white_leds:
                    leds.set_rgb(led, [1.0, 1.0, 1.0])
                leds.set_rgb(3, [1.0, 0.0, 0.0])
                leds.set_rgb(4, [1.0, 0.0, 0.0])
            elif mode == "obstacle":
                for led in driving_leds:
                    leds.set_rgb(led, [1.0, 0.0, 0.0])
            elif mode == "turn_right":
                leds.set_rgb(2, [1.0, 0.6, 0.0])
                leds.set_rgb(3, [1.0, 0.6, 0.0])
                leds.set_rgb(0, [0.0, 0.0, 0.0])
                leds.set_rgb(1, [0.0, 0.0, 0.0])
                leds.set_rgb(4, [0.0, 0.0, 0.0])
            elif mode == "turn_left":
                leds.set_rgb(0, [1.0, 0.6, 0.0])
                leds.set_rgb(4, [1.0, 0.6, 0.0])
                leds.set_rgb(1, [0.0, 0.0, 0.0])
                leds.set_rgb(2, [0.0, 0.0, 0.0])
                leds.set_rgb(3, [0.0, 0.0, 0.0])
        except Exception:
            pass

    def _update_obstacle(self, detections, w, h, leds):
        blocking = [d for d in detections if self._is_obstacle(d, w, h)]
        if blocking:
            self._obstacle_streak += 1
            self._clear_streak     = 0
        else:
            self._clear_streak    += 1
            self._obstacle_streak  = 0

        if self._obstacle_stopped:
            if self._clear_streak >= OBSTACLE_CLEAR_FRAMES:
                self._obstacle_stopped = False
                print("[Agent] Obstacle cleared — resuming", flush=True)
                self._apply_leds(leds, "driving")
        elif self._obstacle_streak >= OBSTACLE_STOP_FRAMES:
            self._obstacle_stopped = True
            labels = sorted({CLASS_NAMES.get(d[2], str(d[2])) for d in blocking})
            print(f"[Agent] Obstacle ahead ({', '.join(labels)}) — stopping", flush=True)
            self._apply_leds(leds, "obstacle")
        return self._obstacle_stopped

    def _transition(self, new_state):
        print(f"[Agent] {self.state} → {new_state}", flush=True)
        self.state = new_state

    def _advance_node(self):
        if self.current_route is None:
            return
        path    = self.current_route.get("path", [])
        current = server.current_node
        try:
            idx = path.index(current)
        except ValueError:
            return
        if idx + 1 < len(path):
            server.current_node = path[idx + 1]
            print(f"[Agent] Node advanced: {current} → {server.current_node}", flush=True)

    def _next_route_node(self):
        if self.current_route is None:
            return None
        path = self.current_route.get("path", [])
        try:
            idx = path.index(server.current_node)
        except ValueError:
            return None
        if idx + 1 < len(path):
            return path[idx + 1]
        return None

    def _complete_at_goal(self, wheels, leds, reason):
        if self._next_route_node() == server.goal_node:
            self._advance_node()
        wheels.set_wheels_speed(0.0, 0.0)
        self._apply_leds(leds, "red_stop")
        self._red_window.clear()
        self._driving_frames = 0
        print(f"[Agent] Goal reached at node {server.current_node} ({reason})", flush=True)
        self._transition("completed")
        return False

    def _red_vote(self, detected):
        self._red_window.append(1 if detected else 0)
        if len(self._red_window) < RED_WINDOW_SIZE:
            return False
        return (sum(self._red_window) / RED_WINDOW_SIZE) >= RED_VOTE_THRESH

    def update(self, frame_bgr, wheels, leds, current_node, goal_node):
        global debug_frame
        red_mask   = None
        fsm_phase  = None
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
                self._driving_frames = -RED_REARM_FRAMES
                self._red_window.clear()
                self.lane_follower._prev_error = 0.0
                self.lane_follower._filtered_error = 0.0
                self._transition("driving")

            # Build debug frame using live detection every frame, including during crossing
            if frame_bgr is not None:
                mask_y, mask_w, red_mask_cross = None, None, None
                try:
                    from tasks.visual_lane_servoing.packages.visual_servoing_activity import (
                        detect_lane_markings,
                    )
                    _my, _mw = detect_lane_markings(frame_bgr)
                    mask_y = (_my * 255).astype(np.uint8)
                    mask_w = (_mw * 255).astype(np.uint8)
                except Exception as e:
                    print(f"[Debug] detect_lane_markings error during crossing: {e}", flush=True)

                try:
                    _, red_mask_cross = detect_red_line(frame_bgr)
                except Exception:
                    pass

                debug_frame = build_debug_frame(
                    frame_bgr, mask_y, mask_w, red_mask_cross,
                    self.state, fsm_phase,
                    self.lane_follower._prev_error,  # real error, not hardcoded 0.0
                )
            return True

        if self.state == "driving":
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            if self.detector is not None:
                with self._det_lock:
                    self._det_frame  = frame_rgb
                    detections       = list(self._detections)
                h, w = frame_bgr.shape[:2]
                if self._update_obstacle(detections, w, h, leds):
                    wheels.set_wheels_speed(0.0, 0.0)
                    debug_frame = build_debug_frame(
                        draw_detections(frame_bgr, detections),
                        None, None, None, self.state, "obstacle", 0.0,
                    )
                    return True

            self._apply_leds(leds, "driving")
            left, right = self.lane_follower.compute_commands(frame_rgb)

            di      = self.lane_follower.last_debug_info
            no_lane = (
                (di.get("total_lane_pixels", 0) < di.get("detection_threshold", 50))
                if di else False
            )
            if no_lane:
                left  = EXIT_SPEED
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
                        print(f"[Agent] First red line at node {server.current_node}", flush=True)
                        self.current_route = dijkstra(
                            server.current_node, goal_node, self._current_heading
                        )
                        self.current_route["start"] = server.current_node
                        print(f"[Agent] Route: {self.current_route['path']}", flush=True)
                        print(f"[Agent] Edges: {self.current_route['edges']}", flush=True)
                        self._route_initialized = True
                    else:
                        self._advance_node()

                    if server.current_node == goal_node:
                        return self._complete_at_goal(
                            wheels, leds, "route node reached on red line"
                        )

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
                raw_bgr     = draw_detections(frame_bgr, detections),
                mask_yellow = di.get("yellow_mask"),
                mask_white  = di.get("white_mask"),
                mask_red    = red_mask,
                state       = self.state,
                sub         = fsm_phase,
                error       = self.lane_follower._prev_error,
            )
        return True


agent = NavigationAgent()


# ============================================================================
# MAIN LOOP
# ============================================================================

def main(camera, wheels, leds, stop_event, server_module=None):
    global server, debug_frame, latest_camera_frame, latest_camera_frame_time

    if server_module is not None:
        server = server_module

    start_dir = getattr(server, "start_direction", "E")
    agent.reset(start_dir)
    agent._apply_leds(leds, "driving")
    debug_frame = None

    print(
        f"[Agent] Started — Start: {server.current_node}  "
        f"Goal: {server.goal_node}  Heading: {agent._current_heading}",
        flush=True,
    )

    def _publish_camera_frame():
        global latest_camera_frame, latest_camera_frame_time
        try:
            if hasattr(camera, "read_rgb"):
                ok, frame_rgb = camera.read_rgb()
                if not ok or frame_rgb is None:
                    return None
                frame_bgr_live = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ok, frame_bgr_live = camera.read()
                if not ok or frame_bgr_live is None:
                    return None
                if len(frame_bgr_live.shape) == 3 and frame_bgr_live.shape[2] == 4:
                    frame_bgr_live = frame_bgr_live[:, :, :3]
            latest_camera_frame = frame_bgr_live.copy()
            latest_camera_frame_time = time.time()
            return frame_bgr_live
        except Exception:
            return None

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal  = server.goal_node

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

            latest_camera_frame = frame_bgr.copy()
            latest_camera_frame_time = time.time()

            should_continue = agent.update(frame_bgr, wheels, leds, start, goal)

            if not should_continue:
                print("[Agent] Route complete — dancing", flush=True)
                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                pause_end = time.time() + 2.0
                while time.time() < pause_end and not stop_event.is_set():
                    _publish_camera_frame()
                    time.sleep(0.05)

                end_time     = time.time() + 4.0
                step         = 0
                dance_colors = [
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0],
                ]
                while time.time() < end_time and not stop_event.is_set():
                    l, r = (0.8, -0.8) if step % 2 == 0 else (-0.8, 0.8)
                    _publish_camera_frame()
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