import unittest

from raspberry_pi.main import (
    RuntimeConfig,
    SharedVisionState,
    _dispatch_gimbal,
    _run_control_loop,
    _to_vision_target,
)
from raspberry_pi.planning.types import GimbalOutput


class _FakePlanner:
    def __init__(self):
        self.calls = []

    def update(self, target, dt_s):
        self.calls.append((target, dt_s))
        return (
            {"cmd": "move", "v": 0.1, "w": 0.0},
            GimbalOutput(
                pan_delta=1.0, tilt_delta=0.5, pan_abs=10.0, tilt_abs=95.0
            ),
        )


class _FakeComm:
    def __init__(self):
        self.sent: list[dict] = []
        self.stop_called = 0
        self.status_called = 0

    def send_message(self, msg):
        self.sent.append(msg)

    def send_stop(self):
        self.stop_called += 1

    def request_status(self):
        self.status_called += 1
        return {"status": "ok"}


class _FailingComm(_FakeComm):
    def send_message(self, msg):
        raise RuntimeError("write failed")


class _FakeGimbalHW:
    def __init__(self):
        self.writes: list[tuple[float, float]] = []
        self.cleaned = False

    def write(self, pan_delta: float, tilt_abs: float) -> None:
        self.writes.append((pan_delta, tilt_abs))

    def cleanup(self) -> None:
        self.cleaned = True


class _FailingGimbalHW(_FakeGimbalHW):
    def write(self, pan_delta: float, tilt_abs: float) -> None:
        raise RuntimeError("pwm failed")


class _AliveOnce:
    def __init__(self):
        self._count = 0

    def __call__(self):
        self._count += 1
        return self._count <= 1


class TestMainPipeline(unittest.TestCase):
    def test_to_vision_target_accepts_valid_pose_info(self):
        target = _to_vision_target(
            {
                "x_error_norm": 0.2,
                "y_error_norm": -0.1,
                "height_norm": 0.4,
                "width_norm": 0.2,
                "confidence": 0.9,
            }
        )
        self.assertIsNotNone(target)
        self.assertEqual(target.x_error_norm, 0.2)
        self.assertEqual(target.y_error_norm, -0.1)

    def test_to_vision_target_rejects_invalid_fields(self):
        target = _to_vision_target(
            {
                "x_error_norm": 2.0,
                "y_error_norm": -0.1,
                "height_norm": 0.4,
                "width_norm": 0.2,
            }
        )
        self.assertIsNone(target)

    def test_control_loop_sends_move_and_writes_gimbal(self):
        planner = _FakePlanner()
        comm = _FakeComm()
        gimbal_hw = _FakeGimbalHW()
        shared = SharedVisionState()
        runtime = RuntimeConfig(port="/dev/null", status_interval_s=0.0)

        now_values = iter([0.0, 0.0, 0.01, 0.015])

        def now_fn():
            return next(now_values)

        target = _to_vision_target(
            {
                "x_error_norm": 0.1,
                "y_error_norm": 0.0,
                "height_norm": 0.3,
                "width_norm": 0.2,
            }
        )
        shared.update(target, 0.0)

        _run_control_loop(
            planner=planner,
            comm=comm,
            gimbal_hw=gimbal_hw,
            shared=shared,
            runtime=runtime,
            vision_alive=_AliveOnce(),
            on_fatal_error=lambda exc: None,
            now_fn=now_fn,
            sleep_fn=lambda _: None,
        )

        self.assertEqual(len(planner.calls), 1)
        self.assertEqual(len(comm.sent), 1)
        self.assertEqual(comm.sent[0]["cmd"], "move")
        self.assertEqual(len(gimbal_hw.writes), 1)
        self.assertEqual(gimbal_hw.writes[0], (1.0, 95.0))

    def test_control_loop_returns_when_vision_fails(self):
        planner = _FakePlanner()
        comm = _FakeComm()
        gimbal_hw = _FakeGimbalHW()
        shared = SharedVisionState()
        runtime = RuntimeConfig(port="/dev/null", status_interval_s=0.0)

        shared.set_vision_failed(RuntimeError("camera failed"))

        _run_control_loop(
            planner=planner,
            comm=comm,
            gimbal_hw=gimbal_hw,
            shared=shared,
            runtime=runtime,
            vision_alive=_AliveOnce(),
            on_fatal_error=lambda exc: None,
            now_fn=lambda: 0.0,
            sleep_fn=lambda _: None,
        )

        self.assertEqual(comm.stop_called, 0)
        self.assertEqual(len(planner.calls), 0)

    def test_control_loop_returns_when_move_send_fails(self):
        planner = _FakePlanner()
        comm = _FailingComm()
        gimbal_hw = _FakeGimbalHW()
        shared = SharedVisionState()
        runtime = RuntimeConfig(port="/dev/null", status_interval_s=0.0)

        target = _to_vision_target(
            {
                "x_error_norm": 0.1,
                "y_error_norm": 0.0,
                "height_norm": 0.3,
                "width_norm": 0.2,
            }
        )
        shared.update(target, 0.0)

        _run_control_loop(
            planner=planner,
            comm=comm,
            gimbal_hw=gimbal_hw,
            shared=shared,
            runtime=runtime,
            vision_alive=_AliveOnce(),
            on_fatal_error=lambda exc: None,
            now_fn=lambda: 0.0,
            sleep_fn=lambda _: None,
        )

        self.assertEqual(len(planner.calls), 1)

    def test_control_loop_returns_when_gimbal_write_fails(self):
        planner = _FakePlanner()
        comm = _FakeComm()
        gimbal_hw = _FailingGimbalHW()
        shared = SharedVisionState()
        runtime = RuntimeConfig(port="/dev/null", status_interval_s=0.0)

        target = _to_vision_target(
            {
                "x_error_norm": 0.1,
                "y_error_norm": 0.0,
                "height_norm": 0.3,
                "width_norm": 0.2,
            }
        )
        shared.update(target, 0.0)

        _run_control_loop(
            planner=planner,
            comm=comm,
            gimbal_hw=gimbal_hw,
            shared=shared,
            runtime=runtime,
            vision_alive=_AliveOnce(),
            on_fatal_error=lambda exc: None,
            now_fn=lambda: 0.0,
            sleep_fn=lambda _: None,
        )

        self.assertEqual(comm.stop_called, 1)

    def test_dispatch_gimbal_calls_hardware_write(self):
        hw = _FakeGimbalHW()
        output = GimbalOutput(
            pan_delta=5.0, tilt_delta=-3.0, pan_abs=15.0, tilt_abs=87.0
        )
        _dispatch_gimbal(output, hw)
        self.assertEqual(len(hw.writes), 1)
        self.assertEqual(hw.writes[0], (5.0, 87.0))


if __name__ == "__main__":
    unittest.main()
