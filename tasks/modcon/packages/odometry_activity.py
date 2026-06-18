from typing import Tuple
import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    delta_ticks = ticks - prev_ticks
    alpha = 2 * np.pi / resolution                      # radians per encoder tick
    dphi = delta_ticks * alpha                          # wheel rotation in radians
    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:
    d_left = R * delta_phi_left                         # left wheel arc length
    d_right = R * delta_phi_right                       # right wheel arc length

    d_A = (d_right + d_left) / 2.0                      # average center displacement
    delta_theta = (d_right - d_left) / baseline          # heading change

    x = x_prev + d_A * np.cos(theta_prev)
    y = y_prev + d_A * np.sin(theta_prev)
    theta = theta_prev + delta_theta

    return x, y, theta