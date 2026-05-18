from .config import PlanningConfig
from .types import GimbalOutput, VisionTarget


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _should_accept_integral(raw_delta: float, limited_delta: float, p_d_delta: float) -> bool:
    if raw_delta == limited_delta:
        return True
    return abs(p_d_delta) < abs(limited_delta)


class GimbalController:
    def __init__(self, config: PlanningConfig):
        self._cfg = config
        self._pan_abs = _clamp(config.pan_center, config.pan_min, config.pan_max)
        self._tilt_abs = _clamp(config.tilt_center, config.tilt_min, config.tilt_max)
        self._filtered_x_err = 0.0
        self._filtered_y_err = 0.0
        self._prev_x_err = 0.0
        self._prev_y_err = 0.0
        self._x_integral = 0.0
        self._y_integral = 0.0

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
        self._filtered_x_err = 0.0
        self._filtered_y_err = 0.0
        self._prev_x_err = 0.0
        self._prev_y_err = 0.0
        self._x_integral = 0.0
        self._y_integral = 0.0

    def compute(self, target: VisionTarget, dt_s: float = 0.0) -> GimbalOutput:
        dt_s = max(0.0, dt_s)
        x_err = target.x_error_norm
        y_err = target.y_error_norm

        if abs(x_err) < self._cfg.deadband_x:
            x_err = 0.0
        if abs(y_err) < self._cfg.deadband_y:
            y_err = 0.0

        a_err = _clamp(self._cfg.gimbal_error_alpha, 0.0, 1.0)
        self._filtered_x_err = (a_err * x_err) + ((1.0 - a_err) * self._filtered_x_err)
        self._filtered_y_err = (a_err * y_err) + ((1.0 - a_err) * self._filtered_y_err)

        x_err = 0.0 if abs(self._filtered_x_err) < self._cfg.deadband_x else self._filtered_x_err
        y_err = 0.0 if abs(self._filtered_y_err) < self._cfg.deadband_y else self._filtered_y_err

        if dt_s > 1e-6:
            x_derivative = (x_err - self._prev_x_err) / dt_s
            y_derivative = (y_err - self._prev_y_err) / dt_s
        else:
            x_derivative = 0.0
            y_derivative = 0.0
        self._prev_x_err = x_err
        self._prev_y_err = y_err

        pan_span_half = (self._cfg.pan_max - self._cfg.pan_min) / 2.0
        tilt_span_half = (self._cfg.tilt_max - self._cfg.tilt_min) / 2.0

        next_x_integral = self._x_integral
        next_y_integral = self._y_integral
        if x_err == 0.0:
            next_x_integral = 0.0
        elif dt_s > 1e-6:
            next_x_integral = _clamp(
                self._x_integral + (x_err * dt_s),
                -self._cfg.integral_limit_pan,
                self._cfg.integral_limit_pan,
            )
        if y_err == 0.0:
            next_y_integral = 0.0
        elif dt_s > 1e-6:
            next_y_integral = _clamp(
                self._y_integral + (y_err * dt_s),
                -self._cfg.integral_limit_tilt,
                self._cfg.integral_limit_tilt,
            )

        pan_pd_delta = -(
            (self._cfg.kp_pan * x_err)
            + (self._cfg.kd_pan * x_derivative)
        ) * pan_span_half
        tilt_pd_delta = (
            (self._cfg.kp_tilt * y_err)
            + (self._cfg.kd_tilt * y_derivative)
        ) * tilt_span_half

        raw_pan_delta = -(
            (self._cfg.kp_pan * x_err)
            + (self._cfg.ki_pan * next_x_integral)
            + (self._cfg.kd_pan * x_derivative)
        ) * pan_span_half
        raw_tilt_delta = (
            (self._cfg.kp_tilt * y_err)
            + (self._cfg.ki_tilt * next_y_integral)
            + (self._cfg.kd_tilt * y_derivative)
        ) * tilt_span_half

        prev_pan = self._pan_abs
        prev_tilt = self._tilt_abs

        pan_delta = _clamp(
            raw_pan_delta,
            -self._cfg.max_pan_delta_per_update,
            self._cfg.max_pan_delta_per_update,
        )
        tilt_delta = _clamp(
            raw_tilt_delta,
            -self._cfg.max_tilt_delta_per_update,
            self._cfg.max_tilt_delta_per_update,
        )

        if abs(pan_delta) < self._cfg.min_pan_delta_per_update:
            pan_delta = 0.0
        if abs(tilt_delta) < self._cfg.min_tilt_delta_per_update:
            tilt_delta = 0.0

        if x_err == 0.0 or _should_accept_integral(raw_pan_delta, pan_delta, pan_pd_delta):
            self._x_integral = next_x_integral
        if y_err == 0.0 or _should_accept_integral(raw_tilt_delta, tilt_delta, tilt_pd_delta):
            self._y_integral = next_y_integral

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
