import unittest

from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.follow_controller import FollowController
from raspberry_pi.planning.types import VisionTarget


class TestFollowController(unittest.TestCase):
    def setUp(self):
        self.cfg = PlanningConfig(
            target_height_norm=0.45,
            kp_angle=1.0,
            kp_distance=1.0,
            v_max=0.5,
            w_max=0.8,
        )
        self.controller = FollowController(self.cfg)

    def test_turns_left_when_pan_abs_negative(self):
        target = VisionTarget(
            x_error_norm=0.0,
            y_error_norm=0.0,
            height_norm=0.45,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target, pan_abs=-54.0)
        self.assertEqual(cmd["cmd"], "move")
        self.assertLess(cmd["w"], 0.0)

    def test_turns_right_when_pan_abs_positive(self):
        target = VisionTarget(
            x_error_norm=0.0,
            y_error_norm=0.0,
            height_norm=0.45,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target, pan_abs=67.5)
        self.assertGreater(cmd["w"], 0.0)

    def test_moves_forward_when_target_far(self):
        target = VisionTarget(
            x_error_norm=0.0,
            y_error_norm=0.0,
            height_norm=0.30,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target)
        self.assertGreater(cmd["v"], 0.0)

    def test_zero_pan_abs_gives_zero_w(self):
        target = VisionTarget(
            x_error_norm=0.02,
            y_error_norm=0.0,
            height_norm=0.45,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target, pan_abs=0.0)
        self.assertEqual(cmd["w"], 0.0)

    def test_w_maps_from_pan_abs_normalized(self):
        target = VisionTarget(
            x_error_norm=0.0,
            y_error_norm=0.0,
            height_norm=0.45,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target, pan_abs=135.0)
        self.assertEqual(cmd["w"], 0.8)

    def test_velocity_is_clamped(self):
        target = VisionTarget(
            x_error_norm=0.0,
            y_error_norm=0.0,
            height_norm=0.0,
            width_norm=0.2,
            confidence=0.9,
        )
        cmd = self.controller.compute(target)
        self.assertLessEqual(abs(cmd["v"]), self.cfg.v_max)


if __name__ == "__main__":
    unittest.main()
