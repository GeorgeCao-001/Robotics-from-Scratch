from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GimbalConfig:
    pan_pin: int = 17
    tilt_pin: int = 27
    pwm_frequency_hz: float = 50.0
    pan_stop_angle: float = 90.0
    pan_max_speed_offset: float = 45.0
    pan_full_speed_delta: float = 180.0


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
        self._pan_pwm.start(self._angle_to_duty(self._cfg.pan_stop_angle))
        self._tilt_pwm.start(self._angle_to_duty(90.0))
        self._setup_done = True

    def write(self, pan_delta: float, tilt_angle: float) -> None:
        self.setup()
        if self._pan_pwm is not None:
            speed = _clamp(
                pan_delta / self._cfg.pan_full_speed_delta,
                -1.0,
                1.0,
            )
            servo_pan = _clamp(
                self._cfg.pan_stop_angle + speed * self._cfg.pan_max_speed_offset,
                0.0,
                180.0,
            )
            self._pan_pwm.ChangeDutyCycle(self._angle_to_duty(servo_pan))
        if self._tilt_pwm is not None:
            servo_tilt = _clamp(tilt_angle, 0.0, 180.0)
            self._tilt_pwm.ChangeDutyCycle(self._angle_to_duty(servo_tilt))

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
    def _angle_to_duty(angle: float) -> float:
        return 2.5 + (angle / 180.0) * 10.0
