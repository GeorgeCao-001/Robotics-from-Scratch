import unittest

from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import VisionTarget


class TestPlanner(unittest.TestCase):
    def setUp(self):
        cfg = PlanningConfig(
            smoothing_alpha_gimbal=1.0,
            smoothing_alpha_move=1.0,
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
        cmds = self.planner.update(target, dt_s=0.03)
        self.assertEqual(len(cmds), 2)
        self.assertEqual(cmds[0]["cmd"], "move")
        self.assertEqual(cmds[1]["cmd"], "gimbal")

    def test_short_loss_keeps_last_commands(self):
        target = VisionTarget(
            x_error_norm=0.3,
            y_error_norm=0.0,
            height_norm=0.30,
            width_norm=0.2,
            confidence=0.9,
        )
        self.planner.update(target, dt_s=0.03)
        second = self.planner.update(None, dt_s=0.1)
        self.assertEqual(second[0]["cmd"], "move")
        self.assertEqual(second[1]["cmd"], "gimbal")
        self.assertEqual(second[1]["pan"], 0.0)
        self.assertEqual(second[1]["tilt"], 0.0)

    def test_long_loss_sends_stop(self):
        target = VisionTarget(
            x_error_norm=0.1,
            y_error_norm=0.0,
            height_norm=0.40,
            width_norm=0.2,
            confidence=0.9,
        )
        self.planner.update(target, dt_s=0.03)
        cmds = self.planner.update(None, dt_s=0.6)
        self.assertEqual(len(cmds), 2)
        self.assertEqual(cmds[0]["cmd"], "move")
        self.assertEqual(cmds[0]["v"], 0.0)
        self.assertEqual(cmds[0]["w"], 0.0)
        self.assertEqual(cmds[1]["cmd"], "gimbal")
        self.assertEqual(cmds[1]["pan"], 0.0)
        self.assertEqual(cmds[1]["tilt"], 0.0)


if __name__ == "__main__":
    unittest.main()
