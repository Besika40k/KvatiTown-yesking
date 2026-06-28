from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}

LANE_FOLLOWING = 'LANE_FOLLOWING'
OBSTACLE_PRESENT = 'OBSTACLE_PRESENT'
DECELERATING = 'DECELERATING'

_state = LANE_FOLLOWING
_decel_frame = 0
_clear_count = 0
_stop_reason = ''


def _get_best_detection(detections: List[Detection], img_size: int):
    best = None
    best_bottom = -1
    for bbox, score, cls_id in detections:
        _, _, _, y2 = bbox
        if y2 > best_bottom:
            best_bottom = y2
            best = (bbox, score, cls_id)
    return best


def _is_in_opposing_lane(bbox, frame_width, cls_id) -> bool:
    if cls_id == 1:
        return False
    x1, _, x2, _ = bbox
    cx = (x1 + x2) / 2
    lane_center = frame_width * 0.35
    if cx < lane_center:
        return True
    return False


def should_stop(detections: List[Detection], frame_width: int, frame_height: int) -> Tuple[bool, str]:
    global _state, _decel_frame, _clear_count, _stop_reason

    RELEASE_FRAMES = 5
    STOP_Y = 0.65
    WARN_Y = 0.40

    filtered = [d for d in detections if not _is_in_opposing_lane(d[0], frame_width, d[2])]

    if _state == LANE_FOLLOWING:
        if not filtered:
            return False, ''

        best = _get_best_detection(filtered, img_size=None)  # img_size unused here
        _, _, _, y2 = best[0]
        cls_id = best[2]

        if y2 > STOP_Y * frame_height:
            _state = OBSTACLE_PRESENT
            _stop_reason = f'{class_names.get(cls_id, str(cls_id))}_ahead'
            return True, _stop_reason

        if y2 > WARN_Y * frame_height:
            _state = DECELERATING
            _decel_frame = 0
            _stop_reason = f'slowing_for_{class_names.get(cls_id, str(cls_id))}'
            return False, _stop_reason

        return False, ''

    elif _state == DECELERATING:
        if not filtered:
            _state = LANE_FOLLOWING
            return False, ''

        _decel_frame += 1
        best = _get_best_detection(filtered, img_size=None)
        _, _, _, y2 = best[0]
        cls_id = best[2]

        if y2 > STOP_Y * frame_height or _decel_frame > 10:
            _state = OBSTACLE_PRESENT
            _stop_reason = f'{class_names.get(cls_id, str(cls_id))}_ahead'
            return True, _stop_reason

        return False, _stop_reason

    else:
        if not filtered:
            _clear_count += 1
            if _clear_count >= RELEASE_FRAMES:
                _state = LANE_FOLLOWING
                _clear_count = 0
                return False, ''
            return True, _stop_reason

        _clear_count = 0
        return True, _stop_reason
