import unittest

from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.gimbal_controller import GimbalController
from raspberry_pi.planning.types import VisionTarget


class TestGimbalController(unittest.TestCase):
    def test_zero_error_outputs_zero_deltas(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)
        target = VisionTarget(0.0, 0.0, 0.4, 0.2, 1.0)
        output = controller.compute(target)
        self.assertEqual(output.pan_delta, 0.0)
        self.assertEqual(output.tilt_delta, 0.0)
        self.assertEqual(output.pan_abs, 0.0)
        self.assertEqual(output.tilt_abs, 0.0)

    def test_tilt_delta_is_clamped_at_boundaries(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=2.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        top_target = VisionTarget(0.0, -1.0, 0.4, 0.2, 1.0)
        output1 = controller.compute(top_target)
        self.assertEqual(output1.tilt_delta, 90.0)
        self.assertEqual(output1.tilt_abs, 90.0)

        output2 = controller.compute(top_target)
        self.assertEqual(output2.tilt_delta, 0.0)
        self.assertEqual(output2.tilt_abs, 90.0)

        bottom_target = VisionTarget(0.0, 1.0, 0.4, 0.2, 1.0)
        output3 = controller.compute(bottom_target)
        self.assertEqual(output3.tilt_delta, -180.0)
        self.assertEqual(output3.tilt_abs, -90.0)

        output4 = controller.compute(bottom_target)
        self.assertEqual(output4.tilt_delta, 0.0)
        self.assertEqual(output4.tilt_abs, -90.0)

    def test_pan_abs_clamps_not_wraps(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        target = VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0)
        output1 = controller.compute(target)
        self.assertEqual(output1.pan_abs, 135.0)

        output2 = controller.compute(target)
        self.assertEqual(output2.pan_abs, 135.0)

    def test_pan_and_tilt_output_deltas(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        target = VisionTarget(-1.0, -1.0, 0.4, 0.2, 1.0)
        output = controller.compute(target)
        self.assertEqual(output.pan_delta, -135.0)
        self.assertEqual(output.tilt_delta, 90.0)
        self.assertEqual(output.pan_abs, -135.0)
        self.assertEqual(output.tilt_abs, 90.0)

    def test_gimbal_smoothing_applies(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=0.5)
        controller = GimbalController(cfg)
        target = VisionTarget(1.0, -1.0, 0.4, 0.2, 1.0)
        output = controller.compute(target)

        self.assertEqual(output.pan_delta, 67.5)
        self.assertEqual(output.tilt_delta, 45.0)
        self.assertEqual(output.pan_abs, 67.5)
        self.assertEqual(output.tilt_abs, 45.0)

    def test_pan_abs_reaches_negative_limit(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        controller.compute(VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0))
        controller.compute(VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0))
        controller.compute(VisionTarget(-1.0, 0.0, 0.4, 0.2, 1.0))
        controller.compute(VisionTarget(-1.0, 0.0, 0.4, 0.2, 1.0))

        self.assertEqual(controller.pan_abs, -135.0)

    def test_centered_target_holds_current_angles(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)
        controller.compute(VisionTarget(0.5, -0.5, 0.4, 0.2, 1.0))

        output = controller.compute(VisionTarget(0.0, 0.0, 0.4, 0.2, 1.0))

        self.assertEqual(output.pan_delta, 0.0)
        self.assertEqual(output.tilt_delta, 0.0)
        self.assertEqual(output.pan_abs, 67.5)
        self.assertEqual(output.tilt_abs, 45.0)

    def test_pan_abs_and_tilt_abs_properties_exposed(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)
        controller.compute(VisionTarget(0.3, -0.2, 0.4, 0.2, 1.0))

        self.assertIsInstance(controller.pan_abs, float)
        self.assertIsInstance(controller.tilt_abs, float)

    def test_reset_brings_angles_to_zero(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)
        controller.compute(VisionTarget(1.0, -1.0, 0.4, 0.2, 1.0))
        controller.reset()
        self.assertEqual(controller.pan_abs, 0.0)
        self.assertEqual(controller.tilt_abs, 0.0)

    def test_reset_uses_configured_center_angles(self):
        cfg = PlanningConfig(
            pan_center=10.0,
            tilt_center=-5.0,
            kp_pan=1.0,
            kp_tilt=1.0,
            smoothing_alpha_gimbal=1.0,
        )
        controller = GimbalController(cfg)
        self.assertEqual(controller.pan_abs, 10.0)
        self.assertEqual(controller.tilt_abs, -5.0)
        controller.compute(VisionTarget(1.0, -1.0, 0.4, 0.2, 1.0))
        controller.reset()
        self.assertEqual(controller.pan_abs, 10.0)
        self.assertEqual(controller.tilt_abs, -5.0)

    def test_deadband_suppresses_small_errors(self):
        cfg = PlanningConfig(
            kp_pan=1.0,
            kp_tilt=1.0,
            smoothing_alpha_gimbal=1.0,
            deadband_x=0.1,
            deadband_y=0.1,
        )
        controller = GimbalController(cfg)
        target = VisionTarget(0.05, -0.05, 0.4, 0.2, 1.0)
        output = controller.compute(target)
        self.assertEqual(output.pan_delta, 0.0)
        self.assertEqual(output.tilt_delta, 0.0)


if __name__ == "__main__":
    unittest.main()
