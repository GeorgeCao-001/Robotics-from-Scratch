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


class GimbalCommand(TypedDict):
    cmd: str
    pan: float
    tilt: float
