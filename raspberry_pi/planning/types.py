from dataclasses import dataclass
from typing import TypedDict


@dataclass(frozen=True)
class VisionTarget:
    x_error_norm: float
    y_error_norm: float
    height_norm: float
    width_norm: float
    confidence: float = 1.0


class MoveCommand(TypedDict):
    cmd: str
    v: float
    w: float


@dataclass(frozen=True)
class GimbalOutput:
    pan_delta: float
    tilt_delta: float
    pan_abs: float
    tilt_abs: float
