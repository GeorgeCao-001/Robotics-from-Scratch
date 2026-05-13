from .config import PlanningConfig
from .types import GimbalOutput, VisionTarget


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class GimbalController:
    def __init__(self, config: PlanningConfig):
        self._cfg = config
        self._pan_total_offset = config.pan_front
        self._pan_abs = config.pan_front
        self._tilt_abs = _clamp(config.tilt_center, config.tilt_min, config.tilt_max)
        self._last_pan = 0.0
        self._last_tilt = 0.0

    @property
    def pan_abs(self) -> float:
        return self._pan_abs

    @property
    def tilt_abs(self) -> float:
        return self._tilt_abs

    def _wrap_to_signed_180(self, angle: float) -> float:
        wrapped = ((angle + 180.0) % 360.0) - 180.0
        if wrapped == -180.0:
            return 180.0
        return wrapped

    def reset(self, pan: float | None = None, tilt: float | None = None) -> None:
        pan_target = self._cfg.pan_front if pan is None else pan
        tilt_target = self._cfg.tilt_center if tilt is None else tilt
        self._pan_total_offset = pan_target
        self._pan_abs = self._wrap_to_signed_180(pan_target)
        self._tilt_abs = _clamp(tilt_target, self._cfg.tilt_min, self._cfg.tilt_max)
        self._last_pan = 0.0
        self._last_tilt = 0.0

    def compute(self, target: VisionTarget) -> GimbalOutput:
        x_err = target.x_error_norm
        y_err = target.y_error_norm

        if abs(x_err) < self._cfg.deadband_x:
            x_err = 0.0
        if abs(y_err) < self._cfg.deadband_y:
            y_err = 0.0

        pan_span = 180.0
        tilt_span = self._cfg.tilt_max - self._cfg.tilt_min

        pan = self._cfg.kp_pan * x_err * pan_span
        tilt = -self._cfg.kp_tilt * y_err * tilt_span

        a = self._cfg.smoothing_alpha_gimbal
        pan = a * pan + (1.0 - a) * self._last_pan
        tilt = a * tilt + (1.0 - a) * self._last_tilt

        pan = _clamp(pan, self._cfg.pan_min, self._cfg.pan_max)

        self._pan_total_offset += pan
        self._pan_abs = self._wrap_to_signed_180(self._pan_total_offset)

        tilt_next = _clamp(
            self._tilt_abs + tilt, self._cfg.tilt_min, self._cfg.tilt_max
        )
        tilt = tilt_next - self._tilt_abs
        self._tilt_abs = tilt_next

        self._last_pan = pan
        self._last_tilt = tilt

        return GimbalOutput(
            pan_delta=round(pan, 3),
            tilt_delta=round(tilt, 3),
            pan_abs=round(self._pan_abs, 3),
            tilt_abs=round(self._tilt_abs, 3),
        )
