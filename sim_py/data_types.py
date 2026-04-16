from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypeAlias


Vec3: TypeAlias = tuple[float, float, float]
Quat: TypeAlias = tuple[float, float, float, float]


def zero_vec3() -> Vec3:
    return (0.0, 0.0, 0.0)


def identity_quat() -> Quat:
    return (1.0, 0.0, 0.0, 0.0)


@dataclass(slots=True)
class POSTruth:
    """真实机体位姿与运动状态。"""

    disp_xyz: Vec3 = field(default_factory=zero_vec3)
    att_quat: Quat = field(default_factory=identity_quat)
    vel_xyz: Vec3 = field(default_factory=zero_vec3)
    acc_xyz: Vec3 = field(default_factory=zero_vec3)
    angv_xyz: Vec3 = field(default_factory=zero_vec3)
    anga_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class SensorTruth:
    """传感器参考系下的真实角速度与加速度。"""

    angv_xyz: Vec3 = field(default_factory=zero_vec3)
    acc_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class SensorObs:
    """带噪声的传感器观测值。"""

    angv_xyz: Vec3 = field(default_factory=zero_vec3)
    acc_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class POSEst:
    """估计得到的机体位姿与运动状态。"""

    disp_xyz: Vec3 = field(default_factory=zero_vec3)
    att_quat: Quat = field(default_factory=identity_quat)
    vel_xyz: Vec3 = field(default_factory=zero_vec3)
    acc_xyz: Vec3 = field(default_factory=zero_vec3)
    angv_xyz: Vec3 = field(default_factory=zero_vec3)
    anga_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class CtrlOutput:
    """控制器输出的目标推力与力矩。"""

    thrust_xyz: Vec3 = field(default_factory=zero_vec3)
    torque_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class CtrlCmd:
    """分配给执行器的控制指令。"""

    servo_x: float = 0.0
    servo_y: float = 0.0
    motor_p: float = 0.0
    motor_n: float = 0.0


@dataclass(slots=True)
class ActorOutput:
    """执行器实际输出的力与力矩。"""

    force_xyz: Vec3 = field(default_factory=zero_vec3)
    torque_xyz: Vec3 = field(default_factory=zero_vec3)


@dataclass(slots=True)
class SensorErr:
    """传感器安装误差与建模误差。"""

    disp_xyz: Vec3 = field(default_factory=zero_vec3)
    disp_err: float = 0.0
    att_quat: Quat = field(default_factory=identity_quat)
    att_err: float = 0.0


@dataclass(slots=True)
class SimulationTarget:
    """控制器使用的目标位姿。"""

    disp_xyz: Vec3 = field(default_factory=zero_vec3)
    att_quat: Quat = field(default_factory=identity_quat)
    vel_xyz: Vec3 = field(default_factory=zero_vec3)
    angv_xyz: Vec3 = field(default_factory=zero_vec3)
