from .config import PlanningConfig
from .follow_controller import FollowController
from .gimbal_controller import GimbalController
from .types import GimbalOutput, MoveCommand, VisionTarget


class Planner:
    def __init__(self, config: PlanningConfig | None = None):
        self._cfg = config or PlanningConfig()
        self._follow = FollowController(self._cfg)
        self._gimbal = GimbalController(self._cfg)
        self._lost_time_s = 0.0
        self._last_move: MoveCommand = {"cmd": "move", "v": 0.0, "w": 0.0}
        self._last_gimbal = GimbalOutput(
            pan_delta=self._cfg.pan_front,
            tilt_delta=self._cfg.tilt_center,
            pan_abs=self._cfg.pan_front,
            tilt_abs=self._cfg.tilt_center,
        )

    def reset(self) -> None:
        self._lost_time_s = 0.0
        self._follow.reset()
        self._gimbal.reset()
        self._last_move = {"cmd": "move", "v": 0.0, "w": 0.0}
        self._last_gimbal = GimbalOutput(
            pan_delta=self._cfg.pan_front,
            tilt_delta=self._cfg.tilt_center,
            pan_abs=self._cfg.pan_front,
            tilt_abs=self._cfg.tilt_center,
        )

    def update(self, target: VisionTarget | None, dt_s: float) -> tuple[MoveCommand, GimbalOutput]:
        dt_s = max(0.0, dt_s)
        zero_gimbal = GimbalOutput(
            pan_delta=0.0,
            tilt_delta=0.0,
            pan_abs=self._gimbal.pan_abs,
            tilt_abs=self._gimbal.tilt_abs,
        )

        if target is None:
            self._lost_time_s += dt_s
            if self._lost_time_s >= self._cfg.lost_timeout_s:
                stop = self._follow.stop_command()
                self._last_move = stop
                self._gimbal.reset()
                self._last_gimbal = GimbalOutput(
                    pan_delta=0.0,
                    tilt_delta=0.0,
                    pan_abs=self._gimbal.pan_abs,
                    tilt_abs=self._gimbal.tilt_abs,
                )
                return (self._last_move, self._last_gimbal)

            self._last_gimbal = zero_gimbal
            return (self._last_move, self._last_gimbal)

        self._lost_time_s = 0.0
        move = self._follow.compute(target)
        gimbal = self._gimbal.compute(target)
        self._last_move = move
        self._last_gimbal = gimbal
        return (move, gimbal)
