from .config import PlanningConfig
from .types import GimbalOutput, VisionTarget


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class GimbalController:
    def __init__(self, config: PlanningConfig):
        self._cfg = config
        self._pan_abs = _clamp(config.pan_center, config.pan_min, config.pan_max)
        self._tilt_abs = _clamp(config.tilt_center, config.tilt_min, config.tilt_max)

    @property
    def pan_abs(self) -> float:
        return self._pan_abs

    @property
    def tilt_abs(self) -> float:
        return self._tilt_abs

    def reset(self, pan: float | None = None, tilt: float | None = None) -> None:
        pan_target = self._cfg.pan_center if pan is None else pan
        tilt_target = self._cfg.tilt_center if tilt is None else tilt
        self._pan_abs = (
            _clamp(pan_target, self._cfg.pan_min, self._cfg.pan_max)
        )
        self._tilt_abs = (
            _clamp(tilt_target, self._cfg.tilt_min, self._cfg.tilt_max)
        )

    def compute(self, target: VisionTarget) -> GimbalOutput:
        x_err = target.x_error_norm
        y_err = target.y_error_norm

        if abs(x_err) < self._cfg.deadband_x:
            x_err = 0.0
        if abs(y_err) < self._cfg.deadband_y:
            y_err = 0.0

        pan_span_half = (self._cfg.pan_max - self._cfg.pan_min) / 2.0
        tilt_span_half = (self._cfg.tilt_max - self._cfg.tilt_min) / 2.0

        raw_pan_delta = self._cfg.kp_pan * x_err * pan_span_half
        raw_tilt_delta = -self._cfg.kp_tilt * y_err * tilt_span_half

        a = self._cfg.smoothing_alpha_gimbal
        prev_pan = self._pan_abs
        prev_tilt = self._tilt_abs

        pan_delta = a * raw_pan_delta
        tilt_delta = a * raw_tilt_delta

        self._pan_abs = round(
            _clamp(prev_pan + pan_delta, self._cfg.pan_min, self._cfg.pan_max), 3
        )
        self._tilt_abs = round(
            _clamp(prev_tilt + tilt_delta, self._cfg.tilt_min, self._cfg.tilt_max),
            3,
        )

        return GimbalOutput(
            pan_delta=round(self._pan_abs - prev_pan, 3),
            tilt_delta=round(self._tilt_abs - prev_tilt, 3),
            pan_abs=self._pan_abs,
            tilt_abs=self._tilt_abs,
        )
