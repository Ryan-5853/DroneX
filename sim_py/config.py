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


@dataclass(slots=True)
class ControllerConfig:
    position_kp: float = 1.0
    attitude_kp: float = 1.0


@dataclass(slots=True)
class AllocatorConfig:
    servo_limit: float = 1.0
    motor_limit: float = 1.0


@dataclass(slots=True)
class ActuatorConfig:
    noise_std_force: float = 0.0
    noise_std_torque: float = 0.0
    response_lag: float = 0.0


@dataclass(slots=True)
class DynamicsConfig:
    mass: float = 1.0
    dt: float = 0.01


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
