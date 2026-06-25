from typing import Tuple
import os
import numpy as np
import cv2
import yaml

_HSV_FILE = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'lane_servoing_hsv_config.yaml')
try:
    with open(_HSV_FILE) as _f:
        _hsv_config = yaml.safe_load(_f) or {}
except FileNotFoundError:
    _hsv_config = {}

_yellow_lower = np.array([_hsv_config.get('yellow_lower_h', 0), _hsv_config.get('yellow_lower_s', 0), _hsv_config.get('yellow_lower_v', 0)])
_yellow_upper = np.array([_hsv_config.get('yellow_upper_h', 0), _hsv_config.get('yellow_upper_s', 0), _hsv_config.get('yellow_upper_v', 0)])

_white_lower = np.array([_hsv_config.get('white_lower_h', 0), _hsv_config.get('white_lower_s', 0), _hsv_config.get('white_lower_v', 0)])
_white_upper = np.array([_hsv_config.get('white_upper_h', 0), _hsv_config.get('white_upper_s', 0), _hsv_config.get('white_upper_v', 0)])


def _clean_lane_mask(mask: np.ndarray, h: int, w: int, fill: bool = False) -> np.ndarray:
    """Keep stable lane-color blobs and remove speckle/noise."""
    roi = np.zeros((h, w), dtype=np.uint8)
    roi[int(h * 0.42):, :] = 255
    mask = cv2.bitwise_and(mask, roi)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 7)) if fill else kernel
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    if fill:
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)), iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cleaned = np.zeros_like(mask)
    min_area = max(25, int(h * w * 0.00012))
    for contour in contours:
        if cv2.contourArea(contour) >= min_area:
            cv2.drawContours(cleaned, [contour], -1, 255, -1)
    return cleaned


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    h, w = image.shape[:2]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(gray, (0, 0), sigmaX=2)

    sobelx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(blurred, cv2.CV_64F, 0, 1)
    gradient_magnitude = np.sqrt(sobelx ** 2 + sobely ** 2)
    edge_support = cv2.dilate(
        (gradient_magnitude > 25).astype(np.uint8) * 255,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
    )

    mask_yellow_color = cv2.inRange(hsv, _yellow_lower, _yellow_upper)
    mask_white_color = cv2.inRange(hsv, _white_lower, _white_upper)

    # Real frames can lose directional Sobel edges on wide/flat paint. HSV is the
    # primary signal; weak edges only reinforce thin borders instead of deleting
    # valid line interiors.
    mask_yellow = cv2.bitwise_or(mask_yellow_color, cv2.bitwise_and(mask_yellow_color, edge_support))
    mask_white = cv2.bitwise_or(mask_white_color, cv2.bitwise_and(mask_white_color, edge_support))

    mask_yellow = _clean_lane_mask(mask_yellow, h, w, fill=True)
    mask_white = _clean_lane_mask(mask_white, h, w)

    # Yellow wins over white if glare makes a yellow line low-saturation.
    yellow_dilated = cv2.dilate(mask_yellow, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask_white = cv2.bitwise_and(mask_white, cv2.bitwise_not(yellow_dilated))

    return (mask_yellow > 0).astype(np.float32), (mask_white > 0).astype(np.float32)


def set_hsv_bounds(yellow_lower, yellow_upper, white_lower, white_upper):
    global _yellow_lower, _yellow_upper, _white_lower, _white_upper
    _yellow_lower = np.array(yellow_lower)
    _yellow_upper = np.array(yellow_upper)
    _white_lower = np.array(white_lower)
    _white_upper = np.array(white_upper)


def get_hsv_bounds():
    return {
        'yellow_lower_h': int(_yellow_lower[0]), 'yellow_upper_h': int(_yellow_upper[0]),
        'yellow_lower_s': int(_yellow_lower[1]), 'yellow_upper_s': int(_yellow_upper[1]),
        'yellow_lower_v': int(_yellow_lower[2]), 'yellow_upper_v': int(_yellow_upper[2]),
        'white_lower_h': int(_white_lower[0]), 'white_upper_h': int(_white_upper[0]),
        'white_lower_s': int(_white_lower[1]), 'white_upper_s': int(_white_upper[1]),
        'white_lower_v': int(_white_lower[2]), 'white_upper_v': int(_white_upper[2]),
    }