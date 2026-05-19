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
    pan_delta: float   # frame-to-frame change, debug only
    tilt_delta: float  # frame-to-frame change, debug only
    pan_abs: float     # horizontal absolute angle, [-135, 135]
    tilt_abs: float    # vertical absolute angle, [-90, 90]
