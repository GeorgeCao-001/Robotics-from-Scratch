import unittest

from raspberry_pi.hardware.gimbal import GimbalConfig, GimbalHardware


class _FakePigpio:
    def __init__(self, connected: bool = True):
        self.pi_instance = _FakePi(connected)

    def pi(self) -> "_FakePi":
        return self.pi_instance


class _FakePi:
    def __init__(self, connected: bool = True):
        self.connected = connected
        self._pulse_widths: list[tuple[int, int]] = []
        self._stopped = False

    def set_servo_pulsewidth(self, pin: int, pulse_width_us: int) -> None:
        self._pulse_widths.append((pin, pulse_width_us))

    def stop(self) -> None:
        self._stopped = True


class TestGimbalHardware(unittest.TestCase):
    def setUp(self):
        self.fake_pigpio = _FakePigpio()

    def test_setup_connects_pigpio_and_writes_initial_positions(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )
        hw.setup()

        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths,
            [(17, 1500), (27, 2000)],
        )

    def test_setup_raises_when_pigpio_daemon_is_not_connected(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=_FakePigpio(connected=False),
        )

        with self.assertRaisesRegex(RuntimeError, "sudo pigpiod"):
            hw.setup()

    def test_setup_is_idempotent(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )
        hw.setup()
        hw.setup()
        self.assertEqual(len(self.fake_pigpio.pi_instance._pulse_widths), 2)

    def test_write_updates_both_servos(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )
        hw.write(45.0, -45.0)

        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths,
            [(17, 1500), (27, 2000), (17, 1833), (27, 1000)],
        )

    def test_write_skips_tiny_angle_changes(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27, min_angle_change_deg=0.5),
            pigpio_module=self.fake_pigpio,
        )

        hw.write(0.2, 45.2)

        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths,
            [(17, 1500), (27, 2000)],
        )

    def test_angle_to_pulse_width_maps_range(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )

        self.assertEqual(hw._angle_to_pulse_width_us(-135.0, -135.0, 135.0), 500)
        self.assertEqual(hw._angle_to_pulse_width_us(0.0, -135.0, 135.0), 1500)
        self.assertEqual(hw._angle_to_pulse_width_us(135.0, -135.0, 135.0), 2500)

    def test_angle_to_pulse_width_maps_tilt_range(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )

        self.assertEqual(hw._angle_to_pulse_width_us(-90.0, -90.0, 90.0), 500)
        self.assertEqual(hw._angle_to_pulse_width_us(0.0, -90.0, 90.0), 1500)
        self.assertEqual(hw._angle_to_pulse_width_us(90.0, -90.0, 90.0), 2500)

    def test_pan_abs_maps_to_position_servo_pulse_width(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )

        hw.write(-135.0, 0.0)
        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths,
            [(17, 1500), (27, 2000), (17, 500), (27, 1500)],
        )

        hw.write(0.0, 0.0)
        self.assertEqual(self.fake_pigpio.pi_instance._pulse_widths[-1], (17, 1500))

        hw.write(135.0, 0.0)
        self.assertEqual(self.fake_pigpio.pi_instance._pulse_widths[-1], (17, 2500))

    def test_tilt_abs_maps_to_position_servo_pulse_width(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )

        hw.write(0.0, 0.0)
        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths,
            [(17, 1500), (27, 2000), (27, 1500)],
        )

        hw.write(0.0, 60.0)
        self.assertEqual(self.fake_pigpio.pi_instance._pulse_widths[-1], (27, 2167))

    def test_clamp_at_boundaries(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )

        hw.write(200.0, 200.0)
        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths[-2:],
            [(17, 2500), (27, 2500)],
        )

        hw.write(-200.0, -200.0)
        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths[-2:],
            [(17, 500), (27, 500)],
        )

    def test_cleanup_stops_servo_pulses_and_closes_pigpio(self):
        hw = GimbalHardware(
            GimbalConfig(pan_pin=17, tilt_pin=27),
            pigpio_module=self.fake_pigpio,
        )
        hw.setup()
        hw.cleanup()

        self.assertEqual(
            self.fake_pigpio.pi_instance._pulse_widths[-2:],
            [(17, 0), (27, 0)],
        )
        self.assertTrue(self.fake_pigpio.pi_instance._stopped)


if __name__ == "__main__":
    unittest.main()
