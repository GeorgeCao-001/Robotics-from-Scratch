from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any


@dataclass(frozen=True)
class GimbalConfig:
    pan_pin: int = 17
    tilt_pin: int = 27
    pan_angle_min: float = -135.0
    pan_angle_max: float = 135.0
    tilt_angle_min: float = -90.0
    tilt_angle_max: float = 90.0
    initial_pan_angle: float = 0.0
    initial_tilt_angle: float = 45.0
    min_pulse_width_us: int = 500
    max_pulse_width_us: int = 2500
    min_angle_change_deg: float = 0.3
    debug: bool = False


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class GimbalHardware:
    def __init__(
        self,
        config: GimbalConfig,
        pigpio_module: Any = None,
    ):
        self._cfg = config
        self._pigpio = pigpio_module
        self._pi: Any = None
        self._setup_done = False
        self._last_debug_log_s = 0.0
        self._last_pan_angle: float | None = None
        self._last_tilt_angle: float | None = None

    def setup(self) -> None:
        if self._setup_done:
            return
        if self._pigpio is None:
            import pigpio

            self._pigpio = pigpio
        self._pi = self._pigpio.pi()
        if not getattr(self._pi, "connected", False):
            self._pi = None
            raise RuntimeError(
                "pigpio daemon is not connected; start it with: sudo pigpiod"
            )
        self._last_pan_angle = _clamp(
            self._cfg.initial_pan_angle,
            self._cfg.pan_angle_min,
            self._cfg.pan_angle_max,
        )
        self._last_tilt_angle = _clamp(
            self._cfg.initial_tilt_angle,
            self._cfg.tilt_angle_min,
            self._cfg.tilt_angle_max,
        )
        self._write_pulse(
            self._cfg.pan_pin,
            self._last_pan_angle,
            self._cfg.pan_angle_min,
            self._cfg.pan_angle_max,
        )
        self._write_pulse(
            self._cfg.tilt_pin,
            self._last_tilt_angle,
            self._cfg.tilt_angle_min,
            self._cfg.tilt_angle_max,
        )
        self._setup_done = True

    def write(self, pan_abs: float, tilt_abs: float) -> None:
        self.setup()
        servo_pan = _clamp(pan_abs, self._cfg.pan_angle_min, self._cfg.pan_angle_max)
        servo_tilt = _clamp(tilt_abs, self._cfg.tilt_angle_min, self._cfg.tilt_angle_max)

        if self._pi is not None and self._should_write(self._last_pan_angle, servo_pan):
            self._write_pulse(
                self._cfg.pan_pin,
                servo_pan,
                self._cfg.pan_angle_min,
                self._cfg.pan_angle_max,
            )
            self._last_pan_angle = servo_pan
        if self._pi is not None and self._should_write(self._last_tilt_angle, servo_tilt):
            self._write_pulse(
                self._cfg.tilt_pin,
                servo_tilt,
                self._cfg.tilt_angle_min,
                self._cfg.tilt_angle_max,
            )
            self._last_tilt_angle = servo_tilt

        if self._cfg.debug:
            now_s = time.monotonic()
            if now_s - self._last_debug_log_s >= 0.5:
                self._last_debug_log_s = now_s
                print(
                    "[GIMBAL] "
                    f"pan_abs={pan_abs:.3f} "
                    f"servo_pan={servo_pan:.1f} "
                    f"tilt_abs={tilt_abs:.3f} "
                    f"servo_tilt={servo_tilt:.1f}"
                )

    def cleanup(self) -> None:
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(self._cfg.pan_pin, 0)
            self._pi.set_servo_pulsewidth(self._cfg.tilt_pin, 0)
            self._pi.stop()
            self._pi = None
        self._setup_done = False
        self._last_pan_angle = None
        self._last_tilt_angle = None

    def _should_write(self, previous: float | None, current: float) -> bool:
        if previous is None:
            return True
        return abs(current - previous) >= self._cfg.min_angle_change_deg

    def _write_pulse(
        self,
        pin: int,
        angle: float,
        angle_min: float,
        angle_max: float,
    ) -> None:
        pulse_width_us = self._angle_to_pulse_width_us(angle, angle_min, angle_max)
        self._pi.set_servo_pulsewidth(pin, pulse_width_us)

    def _angle_to_pulse_width_us(
        self,
        angle: float,
        angle_min: float,
        angle_max: float,
    ) -> int:
        span = angle_max - angle_min
        normalized = (angle - angle_min) / span
        pulse_span = self._cfg.max_pulse_width_us - self._cfg.min_pulse_width_us
        return round(self._cfg.min_pulse_width_us + normalized * pulse_span)
