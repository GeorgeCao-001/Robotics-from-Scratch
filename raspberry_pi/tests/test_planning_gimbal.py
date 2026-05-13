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
        self.assertEqual(output.tilt_abs, 90.0)

    def test_tilt_delta_is_clamped_at_boundaries(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=2.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        top_target = VisionTarget(0.0, -1.0, 0.4, 0.2, 1.0)
        output1 = controller.compute(top_target)
        self.assertEqual(output1.tilt_delta, 90.0)
        self.assertEqual(output1.tilt_abs, 180.0)

        output2 = controller.compute(top_target)
        self.assertEqual(output2.tilt_delta, 0.0)
        self.assertEqual(output2.tilt_abs, 180.0)

        bottom_target = VisionTarget(0.0, 1.0, 0.4, 0.2, 1.0)
        output3 = controller.compute(bottom_target)
        self.assertEqual(output3.tilt_delta, -180.0)
        self.assertEqual(output3.tilt_abs, 0.0)

        output4 = controller.compute(bottom_target)
        self.assertEqual(output4.tilt_delta, 0.0)
        self.assertEqual(output4.tilt_abs, 0.0)

    def test_pan_internal_absolute_wraps_to_signed_range(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        target = VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0)
        output1 = controller.compute(target)
        self.assertEqual(output1.pan_abs, 180.0)

        output2 = controller.compute(target)
        self.assertEqual(output2.pan_abs, 0.0)

    def test_pan_and_tilt_output_deltas(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        target = VisionTarget(-1.0, -1.0, 0.4, 0.2, 1.0)
        output = controller.compute(target)
        self.assertEqual(output.pan_delta, -180.0)
        self.assertEqual(output.tilt_delta, 90.0)
        self.assertEqual(output.pan_abs, 180.0)
        self.assertEqual(output.tilt_abs, 180.0)

    def test_gimbal_smoothing_applies(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=0.5)
        controller = GimbalController(cfg)
        target = VisionTarget(1.0, -1.0, 0.4, 0.2, 1.0)
        output = controller.compute(target)

        self.assertEqual(output.pan_delta, 90.0)
        self.assertEqual(output.tilt_delta, 90.0)
        self.assertEqual(output.pan_abs, 90.0)
        self.assertEqual(output.tilt_abs, 180.0)

    def test_tracks_total_pan_offset_internally(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)

        controller.compute(VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0))
        controller.compute(VisionTarget(1.0, 0.0, 0.4, 0.2, 1.0))
        controller.compute(VisionTarget(-1.0, 0.0, 0.4, 0.2, 1.0))

        self.assertEqual(controller._pan_total_offset, 180.0)

    def test_pan_abs_and_tilt_abs_properties_exposed(self):
        cfg = PlanningConfig(kp_pan=1.0, kp_tilt=1.0, smoothing_alpha_gimbal=1.0)
        controller = GimbalController(cfg)
        controller.compute(VisionTarget(0.3, -0.2, 0.4, 0.2, 1.0))

        self.assertIsInstance(controller.pan_abs, float)
        self.assertIsInstance(controller.tilt_abs, float)


if __name__ == "__main__":
    unittest.main()
