from dataclasses import dataclass


@dataclass
class PlanningConfig:
    kp_angle: float = 0.9
    kp_distance: float = 1.0
    kp_pan: float = 0.0374
    kp_tilt: float = 0.0525
    ki_pan: float = 0.003
    ki_tilt: float = 0.001
    kd_pan: float = 0.0025
    kd_tilt: float = 0.0

    deadband_x: float = 0.2
    deadband_y: float = 0.2
    
    target_height_norm: float = 0.45

    v_max: float = 0.5
    w_max: float = 0.8

    pan_min: float = -135.0
    pan_max: float = 135.0
    tilt_min: float = 0.0
    tilt_max: float = 60.0

    pan_center: float = 0.0
    tilt_center: float = 45.0

    gimbal_error_alpha: float = 0.8
    integral_limit_pan: float = 1.0
    integral_limit_tilt: float = 1.0
    min_pan_delta_per_update: float = 0.0
    min_tilt_delta_per_update: float = 0.0
    max_pan_delta_per_update: float = 10.0
    max_tilt_delta_per_update: float = 5.0
    lost_timeout_s: float = 0.8

    gimbal_pan_pin: int = 17
    gimbal_tilt_pin: int = 27
