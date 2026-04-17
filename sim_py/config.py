from __future__ import annotations

from dataclasses import dataclass, field

from .data_types import POSTruth, SensorErr, SimulationTarget


@dataclass(slots=True)
class SensorConfig:
    noise_std_gyro: float = 0.0
    noise_std_acc: float = 0.0
    install_error: SensorErr = field(default_factory=SensorErr)


@dataclass(slots=True)
class EstimatorConfig:
    enable_model_error: bool = False
    sample_dt: float = 0.01
    acc_correction_gain: float = 2.0
    acc_norm_min: float = 6.0
    acc_norm_max: float = 13.0
    gravity_ref_world: tuple[float, float, float] = (0.0, 0.0, -1.0)


@dataclass(slots=True)
class ControllerConfig:
    position_kp: float = 1.0
    attitude_kp: float = 1.0
    dt: float = 0.01
    mass_estimate: float = 1.0
    gravity_world: tuple[float, float, float] = (0.0, 0.0, -9.81)
    enable_position_loop: bool = True
    enable_attitude_loop: bool = True
    pos_kp_xyz: tuple[float, float, float] = (1.0, 1.0, 1.0)
    vel_kp_xyz: tuple[float, float, float] = (0.8, 0.8, 1.0)
    pos_ki_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pos_integral_limit_xyz: tuple[float, float, float] = (2.0, 2.0, 2.0)
    att_kp_xyz: tuple[float, float, float] = (2.5, 2.5, 1.8)
    rate_kp_xyz: tuple[float, float, float] = (0.35, 0.35, 0.35)
    att_ki_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    att_integral_limit_xyz: tuple[float, float, float] = (0.8, 0.8, 0.8)
    thrust_min: float = 0.0
    thrust_max: float = 15.0
    torque_limit_xyz: tuple[float, float, float] = (8.0, 8.0, 4.0)
    anti_windup_gain: float = 0.2


@dataclass(slots=True)
class AllocatorConfig:
    servo_limit: float = 1.0
    motor_limit: float = 1.0
    vector_tilt_limit_rad: float = 0.5235987755982988
    thrust_to_motor_cmd_gain: float = 1.0
    motor_torque_to_diff_cmd_gain: float = 0.08
    thrust_point_vector_unit_m: tuple[float, float, float] = (0.0, 0.0, -0.55)
    tilt_torque_xy_gain: float = 1.0
    yaw_motor_torque_ratio: float = 0.12
    body_to_vector_dcm: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    vector_torque_to_motor_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    motor_axis_to_vector_torque: tuple[float, float, float] = (0.0, 0.0, 1.0)
    servo_x_roll_sign: float = -1.0
    servo_y_pitch_sign: float = 1.0
    servo_x_mech_gain: float = 1.0
    servo_y_mech_gain: float = 1.0


@dataclass(slots=True)
class ActuatorConfig:
    dt: float = 0.01
    servo_cmd_limit: float = 1.0
    motor_cmd_limit: float = 1.0
    servo_rate_limit: float = 8.0
    motor_rate_limit: float = 20.0
    servo_time_constant: float = 0.05
    motor_time_constant: float = 0.08
    servo_cmd_noise_std: float = 0.0
    motor_cmd_noise_std: float = 0.0
    servo_random_walk_std: float = 0.0
    motor_random_walk_std: float = 0.0
    servo_angle_offset_rad: tuple[float, float] = (0.0, 0.0)
    servo_angle_gain_rad: tuple[float, float] = (0.5235987755982988, 0.5235987755982988)
    vector_to_body_dcm: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    vector_origin_body_m: tuple[float, float, float] = (0.0, 0.0, -0.55)
    thrust_axis_vector_unit: tuple[float, float, float] = (0.0, 0.0, 1.0)
    motor_thrust_gain: tuple[float, float] = (1.0, 1.0)
    motor_thrust_exp: float = 1.0
    motor_axis_torque_gain: tuple[float, float] = (0.08, 0.08)
    motor_axis_vector_unit: tuple[float, float, float] = (0.0, 0.0, 1.0)
    servo_rate_extra_torque_matrix: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] = (
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    )
    noise_std_force: float = 0.0
    noise_std_torque: float = 0.0
    response_lag: float = 0.0


@dataclass(slots=True)
class DynamicsConfig:
    mass: float = 1.0
    dt: float = 0.01
    gravity_world: tuple[float, float, float] = (0.0, 0.0, -9.81)
    center_of_mass_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    thrust_point_body_m: tuple[float, float, float] = (0.0, 0.0, -0.55)
    enable_gravity_lever_torque: bool = True
    gravity_lever_torque_gain: float = 1.0
    inertia_body_diag: tuple[float, float, float] = (0.01, 0.01, 0.02)
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wind_gust_std: float = 0.0
    linear_drag_coeff_world: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quadratic_drag_coeff: float = 0.0
    angular_damping_linear: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_damping_quadratic: float = 0.0
    disturbance_force_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    disturbance_torque_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    disturbance_force_noise_std: float = 0.0
    disturbance_torque_noise_std: float = 0.0


@dataclass(slots=True)
class SimulationConfig:
    sensor: SensorConfig = field(default_factory=SensorConfig)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    allocator: AllocatorConfig = field(default_factory=AllocatorConfig)
    actuator: ActuatorConfig = field(default_factory=ActuatorConfig)
    dynamics: DynamicsConfig = field(default_factory=DynamicsConfig)
    initial_truth: POSTruth = field(default_factory=POSTruth)
    target: SimulationTarget = field(default_factory=SimulationTarget)
