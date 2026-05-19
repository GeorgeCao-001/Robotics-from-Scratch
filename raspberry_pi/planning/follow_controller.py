from .config import PlanningConfig
from .types import MoveCommand, VisionTarget


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class FollowController:
    def __init__(self, config: PlanningConfig):
        self._cfg = config

    def reset(self) -> None:
        pass

    def stop_command(self) -> MoveCommand:
        self.reset()
        return {"cmd": "move", "v": 0.0, "w": 0.0}

    def compute(self, target: VisionTarget, pan_abs: float = 0.0) -> MoveCommand:
        pan_span_half = (self._cfg.pan_max - self._cfg.pan_min) / 2.0
        pan_norm = 0.0 if pan_span_half <= 0.0 else pan_abs / pan_span_half
        raw_w = self._cfg.kp_angle * pan_norm
        raw_w = _clamp(raw_w, -self._cfg.w_max, self._cfg.w_max)

        dist_err = self._cfg.target_height_norm - target.height_norm
        raw_v = self._cfg.kp_distance * dist_err
        raw_v = _clamp(raw_v, -self._cfg.v_max, self._cfg.v_max)

        return {"cmd": "move", "v": round(raw_v, 3), "w": round(raw_w, 3)}
