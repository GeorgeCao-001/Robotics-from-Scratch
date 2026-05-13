from dataclasses import dataclass


@dataclass
class PlanningConfig:
    kp_angle: float = 0.9
    kp_distance: float = 1.0
    kp_pan: float = 1.0
    kp_tilt: float = 1.0

    deadband_x: float = 0.05
    deadband_y: float = 0.05

    target_height_norm: float = 0.45

    v_max: float = 0.5
    w_max: float = 0.8

    pan_min: float = -180.0
    pan_max: float = 180.0
    tilt_min: float = 0.0
    tilt_max: float = 180.0

    pan_front: float = 0.0
    tilt_center: float = 90.0

    smoothing_alpha_move: float = 0.3
    smoothing_alpha_gimbal: float = 0.5
    lost_timeout_s: float = 0.8

    gimbal_pan_pin: int = 17
    gimbal_tilt_pin: int = 27
