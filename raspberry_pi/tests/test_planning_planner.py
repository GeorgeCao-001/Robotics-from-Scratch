import unittest

from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import VisionTarget


class TestPlanner(unittest.TestCase):
    def setUp(self):
        cfg = PlanningConfig(
            lost_timeout_s=0.5,
        )
        self.planner = Planner(cfg)

    def test_tracking_outputs_move_and_gimbal(self):
        target = VisionTarget(
            x_error_norm=0.2,
            y_error_norm=-0.1,
            height_norm=0.35,
            width_norm=0.22,
            confidence=0.95,
        )
        move, gimbal = self.planner.update(target, dt_s=0.03)
        self.assertEqual(move["cmd"], "move")
        self.assertIsNotNone(gimbal)
        self.assertTrue(hasattr(gimbal, "pan_delta"))
        self.assertTrue(hasattr(gimbal, "tilt_delta"))
        self.assertTrue(hasattr(gimbal, "pan_abs"))
        self.assertTrue(hasattr(gimbal, "tilt_abs"))

    def test_short_loss_keeps_last_commands(self):
        target = VisionTarget(
            x_error_norm=0.3,
            y_error_norm=0.0,
            height_norm=0.30,
            width_norm=0.2,
            confidence=0.9,
        )
        self.planner.update(target, dt_s=0.03)
        move, gimbal = self.planner.update(None, dt_s=0.1)
        self.assertEqual(move["cmd"], "move")
        self.assertEqual(gimbal.pan_delta, 0.0)
        self.assertEqual(gimbal.tilt_delta, 0.0)

    def test_long_loss_sends_stop(self):
        target = VisionTarget(
            x_error_norm=0.1,
            y_error_norm=0.0,
            height_norm=0.40,
            width_norm=0.2,
            confidence=0.9,
        )
        self.planner.update(target, dt_s=0.03)
        move, gimbal = self.planner.update(None, dt_s=0.6)
        self.assertEqual(move["cmd"], "move")
        self.assertEqual(move["v"], 0.0)
        self.assertEqual(move["w"], 0.0)
        self.assertEqual(gimbal.pan_delta, 0.0)
        self.assertEqual(gimbal.tilt_delta, 0.0)

    def test_long_loss_keeps_gimbal_position(self):
        target = VisionTarget(
            x_error_norm=0.2,
            y_error_norm=-0.1,
            height_norm=0.40,
            width_norm=0.2,
            confidence=0.9,
        )
        _, tracked_gimbal = self.planner.update(target, dt_s=0.03)

        _, lost_gimbal = self.planner.update(None, dt_s=0.6)

        self.assertEqual(lost_gimbal.pan_abs, tracked_gimbal.pan_abs)
        self.assertEqual(lost_gimbal.tilt_abs, tracked_gimbal.tilt_abs)


if __name__ == "__main__":
    unittest.main()
