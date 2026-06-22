import os
import time
import yaml
import numpy as np
from collections import deque

from tasks.modcon.packages.odometry_activity import delta_phi, pose_estimation
from tasks.modcon.packages.pid_controller import PIDController

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'modcon_config.yaml'
))

CONTROL_DT = 0.1


class ModConAgent:

    def __init__(self, wheels=None, config_path: str = None):
        try:
            with open(config_path or _CONFIG_FILE) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            cfg = {}

        self.v_0        = cfg.get('v_0',                0.3)
        self.R          = cfg.get('radius',             0.0318)
        self.baseline   = cfg.get('baseline',           0.1)
        self.resolution = cfg.get('encoder_resolution', 135)

        self._wheels  = wheels
        self.x = self.y = self.theta = 0.0
        self._prev_l  = 0
        self._prev_r  = 0
        self._pid_e   = 0.0
        self._pid_int = 0.0

    def set_wheels(self, wheels):
        self._wheels = wheels

    @property
    def pose(self) -> tuple:
        return self.x, self.y, self.theta

    def reset_pose(self):
        self.x = self.y = self.theta = 0.0
        if self._wheels and self._wheels.encoders:
            enc = self._wheels.encoders
            self._prev_l = enc.left.ticks
            self._prev_r = enc.right.ticks

    # ── Internals ─────────────────────────────────────────────────────────

    def _update_odometry(self):
        if not self._wheels or not self._wheels.encoders:
            return
        enc = self._wheels.encoders
        dphi_l, self._prev_l = delta_phi(enc.left.ticks,  self._prev_l, self.resolution)
        dphi_r, self._prev_r = delta_phi(enc.right.ticks, self._prev_r, self.resolution)
        self.x, self.y, self.theta = pose_estimation(
            self.R, self.baseline, self.x, self.y, self.theta, dphi_l, dphi_r
        )

    def _ticks_to_dist(self, ticks: float) -> float:
        return (ticks / self.resolution) * 2.0 * np.pi * self.R

    # ── Maneuvers ─────────────────────────────────────────────────────────

    def run_straight(self, distance: float, timeout: float = 30.0):
        enc = self._wheels.encoders
        start_l, start_r = enc.left.ticks, enc.right.ticks
        prev_l,  prev_r  = start_l, start_r
        deadline = time.time() + timeout

        while time.time() < deadline:
            self._update_odometry()
            dist = self._ticks_to_dist(
                ((enc.left.ticks - start_l) + (enc.right.ticks - start_r)) / 2.0
            )
            if abs(distance - dist) < 0.02:
                break

            speed = float(np.clip((distance - dist) * 0.5, -self.v_0, self.v_0))
            if 0 < abs(speed) < self.v_0 * 0.5:
                speed = np.sign(speed) * self.v_0 * 0.5

            # PI tick-balance correction: keeps left/right ticks equal → straight line
            step_diff  = (enc.right.ticks - prev_r) - (enc.left.ticks - prev_l)
            cumul_diff = (enc.right.ticks - start_r) - (enc.left.ticks - start_l)
            omega = -(0.4 * step_diff + 0.02 * cumul_diff)
            prev_l, prev_r = enc.left.ticks, enc.right.ticks

            print(f"[straight] {dist:.3f}/{distance:.2f}m  ω={omega:+.3f}")
            self._wheels.set_velocity(speed, omega)
            time.sleep(CONTROL_DT)

        self._wheels.set_velocity(0.0, 0.0)
        time.sleep(0.3)

    def run_turn(self, degrees: float, timeout: float = 20.0):

        target = self.theta + np.deg2rad(degrees)

        self._pid_e = 0.0
        self._pid_int = 0.0

        deadline = time.time() + timeout

        while time.time() < deadline:

            self._update_odometry()

            _, omega, self._pid_e, self._pid_int = PIDController(
                self.v_0,
                target,
                self.theta,
                self._pid_e,
                self._pid_int,
                CONTROL_DT
            )

            # Wrapped angle error
            error = np.arctan2(
                np.sin(target - self.theta),
                np.cos(target - self.theta)
            )

            # Slow down near target
            if abs(error) < np.deg2rad(10):
                omega *= 0.5

            # Minimum turning speed to avoid stalling
            min_omega = 1.0
            if 0 < abs(omega) < min_omega:
                omega = np.sign(omega) * min_omega

            print(
                f"[turn] "
                f"target={np.rad2deg(target):+.1f}°  "
                f"theta={np.rad2deg(self.theta):+.1f}°  "
                f"err={np.rad2deg(error):+.1f}°  "
                f"omega={omega:+.2f}"
            )

            # Stop condition
            if abs(error) < np.deg2rad(2.0):
                break

            self._wheels.set_velocity(0.0, omega)

            time.sleep(CONTROL_DT)

        self._wheels.set_velocity(0.0, 0.0)

        time.sleep(0.5)

    def run_square(self, side: float):
        start_theta = self.theta
        for i in range(4):
            print(f"[square] side {i + 1}/4")
            if i > 0:
                target = start_theta + i * np.pi / 2
                delta  = np.rad2deg(np.arctan2(np.sin(target - self.theta), np.cos(target - self.theta)))
                self.run_turn(delta)
            self.run_straight(side)
        self._wheels.set_velocity(0.0, 0.0)
