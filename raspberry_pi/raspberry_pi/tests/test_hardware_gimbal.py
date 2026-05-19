import unittest

from raspberry_pi.hardware.gimbal import GimbalConfig, GimbalHardware


class _FakeGPIO:
    BCM = "BCM"
    OUT = "OUT"

    def __init__(self):
        self._mode = None
        self._setups: list[tuple[int, str]] = []
        self._pwms: dict[int, "_FakePWM"] = {}
        self._cleaned = False

    def setmode(self, mode: str) -> None:
        self._mode = mode

    def setup(self, pin: int, direction: str) -> None:
        self._setups.append((pin, direction))

    def PWM(self, pin: int, freq: float) -> "_FakePWM":
        pwm = _FakePWM(pin, freq)
        self._pwms[pin] = pwm
        return pwm

    def cleanup(self) -> None:
        self._cleaned = True


class _FakePWM:
    def __init__(self, pin: int, freq: float):
        self.pin = pin
        self.freq = freq
        self._duty_cycles: list[float] = []
        self._started = False
        self._stopped = False

    def start(self, duty: float) -> None:
        self._started = True
        self._duty_cycles.append(duty)

    def ChangeDutyCycle(self, duty: float) -> None:
        self._duty_cycles.append(duty)

    def stop(self) -> None:
        self._stopped = True


class TestGimbalHardware(unittest.TestCase):
    def setUp(self):
        self.fake_gpio = _FakeGPIO()

    def test_setup_configures_pins_and_pwm(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )
        hw.setup()

        self.assertEqual(self.fake_gpio._mode, "BCM")
        self.assertEqual(len(self.fake_gpio._setups), 2)
        self.assertIn(17, self.fake_gpio._pwms)
        self.assertIn(27, self.fake_gpio._pwms)
        self.assertEqual(self.fake_gpio._pwms[17].freq, 50.0)
        self.assertEqual(self.fake_gpio._pwms[27].freq, 50.0)

    def test_setup_is_idempotent(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )
        hw.setup()
        hw.setup()
        self.assertEqual(len(self.fake_gpio._setups), 2)

    def test_write_updates_both_servos(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )
        hw.write(45.0, -45.0)

        pan_duties = self.fake_gpio._pwms[17]._duty_cycles
        tilt_duties = self.fake_gpio._pwms[27]._duty_cycles

        self.assertGreater(len(pan_duties), 1)
        self.assertGreater(len(tilt_duties), 1)

    def test_angle_to_duty_maps_range(self):
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(-135.0, -135.0, 135.0), 2.5
        )
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(0.0, -135.0, 135.0), 7.5
        )
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(135.0, -135.0, 135.0), 12.5
        )

    def test_angle_to_duty_maps_tilt_range(self):
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(-90.0, -90.0, 90.0), 2.5
        )
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(0.0, -90.0, 90.0), 7.5
        )
        self.assertAlmostEqual(
            GimbalHardware._angle_to_duty(90.0, -90.0, 90.0), 12.5
        )

    def test_pan_abs_maps_to_position_servo_duty(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )

        hw.write(-135.0, 0.0)
        pan_duties = self.fake_gpio._pwms[17]._duty_cycles
        self.assertAlmostEqual(pan_duties[0], 7.5, places=3)
        self.assertAlmostEqual(pan_duties[1], 2.5, places=3)

        hw.write(0.0, 0.0)
        pan_duties_1 = self.fake_gpio._pwms[17]._duty_cycles
        self.assertAlmostEqual(pan_duties_1[2], 7.5, places=3)

        hw.write(135.0, 0.0)
        pan_duties_2 = self.fake_gpio._pwms[17]._duty_cycles
        self.assertAlmostEqual(pan_duties_2[3], 12.5, places=3)

    def test_tilt_abs_maps_to_position_servo_duty(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )

        hw.write(0.0, -90.0)
        tilt_duties = self.fake_gpio._pwms[27]._duty_cycles
        self.assertAlmostEqual(tilt_duties[0], 7.5, places=3)
        self.assertAlmostEqual(tilt_duties[1], 2.5, places=3)

        hw.write(0.0, 90.0)
        tilt_duties = self.fake_gpio._pwms[27]._duty_cycles
        self.assertAlmostEqual(tilt_duties[2], 12.5, places=3)

    def test_clamp_at_boundaries(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )

        hw.write(200.0, 200.0)
        pan_duties = self.fake_gpio._pwms[17]._duty_cycles
        tilt_duties = self.fake_gpio._pwms[27]._duty_cycles
        self.assertAlmostEqual(pan_duties[1], 12.5, places=3)
        self.assertAlmostEqual(tilt_duties[1], 12.5, places=3)

        hw.write(-200.0, -200.0)
        pan_duties_2 = self.fake_gpio._pwms[17]._duty_cycles
        tilt_duties_2 = self.fake_gpio._pwms[27]._duty_cycles
        self.assertAlmostEqual(pan_duties_2[2], 2.5, places=3)
        self.assertAlmostEqual(tilt_duties_2[2], 2.5, places=3)

    def test_cleanup_stops_pwm_and_cleans_gpio(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            gpio_module=self.fake_gpio,
        )
        hw.setup()
        hw.cleanup()

        for pwm in self.fake_gpio._pwms.values():
            self.assertTrue(pwm._stopped)
        self.assertTrue(self.fake_gpio._cleaned)


if __name__ == "__main__":
    unittest.main()
