from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any


@dataclass(frozen=True)
class GimbalConfig:
    pan_pin: int = 17
    tilt_pin: int = 27
    pwm_frequency_hz: float = 50.0
    pan_angle_min: float = -135.0
    pan_angle_max: float = 135.0
    tilt_angle_min: float = -90.0
    tilt_angle_max: float = 90.0
    debug: bool = False


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class GimbalHardware:
    def __init__(
        self,
        config: GimbalConfig,
        gpio_module: Any = None,
    ):
        self._cfg = config
        self._gpio = gpio_module
        self._pan_pwm: Any = None
        self._tilt_pwm: Any = None
        self._setup_done = False
        self._last_debug_log_s = 0.0

    def setup(self) -> None:
        if self._setup_done:
            return
        if self._gpio is None:
            import RPi.GPIO as GPIO

            self._gpio = GPIO
        self._gpio.setmode(self._gpio.BCM)
        self._gpio.setup(self._cfg.pan_pin, self._gpio.OUT)
        self._gpio.setup(self._cfg.tilt_pin, self._gpio.OUT)
        self._pan_pwm = self._gpio.PWM(self._cfg.pan_pin, self._cfg.pwm_frequency_hz)
        self._tilt_pwm = self._gpio.PWM(self._cfg.tilt_pin, self._cfg.pwm_frequency_hz)
        self._pan_pwm.start(self._angle_to_duty(0.0, self._cfg.pan_angle_min, self._cfg.pan_angle_max))
        self._tilt_pwm.start(self._angle_to_duty(0.0, self._cfg.tilt_angle_min, self._cfg.tilt_angle_max))
        self._setup_done = True

    def write(self, pan_abs: float, tilt_abs: float) -> None:
        self.setup()
        servo_pan = _clamp(pan_abs, self._cfg.pan_angle_min, self._cfg.pan_angle_max)
        servo_tilt = _clamp(tilt_abs, self._cfg.tilt_angle_min, self._cfg.tilt_angle_max)

        if self._pan_pwm is not None:
            pan_duty = self._angle_to_duty(
                servo_pan, self._cfg.pan_angle_min, self._cfg.pan_angle_max
            )
            self._pan_pwm.ChangeDutyCycle(pan_duty)
        if self._tilt_pwm is not None:
            tilt_duty = self._angle_to_duty(
                servo_tilt, self._cfg.tilt_angle_min, self._cfg.tilt_angle_max
            )
            self._tilt_pwm.ChangeDutyCycle(tilt_duty)

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
        if self._pan_pwm is not None:
            self._pan_pwm.stop()
            self._pan_pwm = None
        if self._tilt_pwm is not None:
            self._tilt_pwm.stop()
            self._tilt_pwm = None
        if self._gpio is not None:
            self._gpio.cleanup()
        self._setup_done = False

    @staticmethod
    def _angle_to_duty(angle: float, angle_min: float, angle_max: float) -> float:
        span = angle_max - angle_min
        normalized = (angle - angle_min) / span
        return 2.5 + normalized * 10.0
