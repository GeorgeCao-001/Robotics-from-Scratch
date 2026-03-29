from .config import PlanningConfig
from .types import MoveCommand, VisionTarget


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class FollowController:
    def __init__(self, config: PlanningConfig):
        self._cfg = config
        self._last_v = 0.0
        self._last_w = 0.0

    def reset(self) -> None:
        self._last_v = 0.0
        self._last_w = 0.0

    def stop_command(self) -> MoveCommand:
        self.reset()
        return {"cmd": "move", "v": 0.0, "w": 0.0}

    def compute(self, target: VisionTarget) -> MoveCommand:
        x_err = target.x_error_norm
        if abs(x_err) < self._cfg.deadband_x:
            x_err = 0.0

        y_err = target.y_error_norm
        if abs(y_err) < self._cfg.deadband_y:
            y_err = 0.0

        # P control for omega
        raw_w = self._cfg.kp_angle * x_err
        raw_w = _clamp(raw_w, -self._cfg.w_max, self._cfg.w_max)

        # P control for v
        dist_err = self._cfg.target_height_norm - target.height_norm
        raw_v = self._cfg.kp_distance * dist_err
        raw_v = _clamp(raw_v, -self._cfg.v_max, self._cfg.v_max)

        # momentum smoother
        a = self._cfg.smoothing_alpha_move
        v = a * raw_v + (1.0 - a) * self._last_v
        w = a * raw_w + (1.0 - a) * self._last_w

        self._last_v = v
        self._last_w = w

        return {"cmd": "move", "v": round(v, 3), "w": round(w, 3)}
