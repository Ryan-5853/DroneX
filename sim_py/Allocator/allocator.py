from __future__ import annotations

import math

from sim_py.config import AllocatorConfig
from sim_py.data_types import CtrlCmd, CtrlOutput, Vec3
from sim_py.math_utils import clamp


class AllocatorModule:
    """将推力/力矩目标映射为具体执行器控制指令。"""

    def __init__(self, config: AllocatorConfig) -> None:
        self.config = config

    def allocate(self, ctrl_output: CtrlOutput) -> CtrlCmd:
        thrust_vector_body = ctrl_output.thrust_xyz
        torque_vector_body = ctrl_output.torque_xyz

        thrust_vector_unit = self._body_to_vector_unit(thrust_vector_body)
        torque_vector_unit = self._body_to_vector_unit(torque_vector_body)

        # 主要通过矢量单元偏转满足XY扭矩，弱化电机强扭矩通道。
        thrust_vector_for_tilt = self._inject_xy_torque_into_thrust(thrust_vector_unit, torque_vector_unit)

        servo_x, servo_y = self._map_thrust_to_servos(thrust_vector_for_tilt)
        motor_p, motor_n = self._map_torque_to_motors(thrust_vector_for_tilt, torque_vector_unit)

        return CtrlCmd(
            servo_x=servo_x,
            servo_y=servo_y,
            motor_p=motor_p,
            motor_n=motor_n,
        )

    def _map_thrust_to_servos(self, thrust_vector_unit: Vec3) -> tuple[float, float]:
        thrust_norm = _norm(thrust_vector_unit)
        if thrust_norm <= 1.0e-9:
            return (0.0, 0.0)

        dir_x = thrust_vector_unit[0] / thrust_norm
        dir_y = thrust_vector_unit[1] / thrust_norm
        dir_z = thrust_vector_unit[2] / thrust_norm

        pitch_rad = math.asin(clamp(-dir_y, -1.0, 1.0))
        roll_rad = math.atan2(dir_x, dir_z)

        # 连杆机械学关系留空：使用可配置的线性占位映射，后续可直接改配置参数。
        servo_x = _roll_to_servo_x(
            roll_rad=roll_rad,
            servo_limit=self.config.servo_limit,
            vector_tilt_limit_rad=self.config.vector_tilt_limit_rad,
            roll_sign=self.config.servo_x_roll_sign,
            mech_gain=self.config.servo_x_mech_gain,
        )
        servo_y = _pitch_to_servo_y(
            pitch_rad=pitch_rad,
            servo_limit=self.config.servo_limit,
            vector_tilt_limit_rad=self.config.vector_tilt_limit_rad,
            pitch_sign=self.config.servo_y_pitch_sign,
            mech_gain=self.config.servo_y_mech_gain,
        )
        return (servo_x, servo_y)

    def _map_torque_to_motors(self, thrust_vector_unit: Vec3, torque_vector_unit: Vec3) -> tuple[float, float]:
        total_thrust_cmd = max(0.0, _norm(thrust_vector_unit) * self.config.thrust_to_motor_cmd_gain)

        # 电机差速只用于弱yaw通道，避免“电机强扭矩”主导姿态控制。
        torque_for_motor = (0.0, 0.0, torque_vector_unit[2] * self.config.yaw_motor_torque_ratio)
        torque_motor_axis = _vector_unit_torque_to_motor_axis(
            torque_for_motor,
            self.config.vector_torque_to_motor_axis,
        )
        diff_cmd = torque_motor_axis * self.config.motor_torque_to_diff_cmd_gain

        motor_p = clamp(total_thrust_cmd + diff_cmd, 0.0, self.config.motor_limit)
        motor_n = clamp(total_thrust_cmd - diff_cmd, 0.0, self.config.motor_limit)

        # 保留一个反向映射占位，便于后续将差速输出扭矩回写到矢量单元系做一致性校核。
        _ = _motor_axis_to_vector_unit_torque(
            motor_p - motor_n,
            self.config.motor_axis_to_vector_torque,
        )
        return (motor_p, motor_n)

    def _body_to_vector_unit(self, v_body: Vec3) -> Vec3:
        return _mat_vec_mul(self.config.body_to_vector_dcm, v_body)

    def _inject_xy_torque_into_thrust(self, thrust_vector_unit: Vec3, torque_vector_unit: Vec3) -> Vec3:
        rz = self.config.thrust_point_vector_unit_m[2]
        if abs(rz) <= 1.0e-6:
            return thrust_vector_unit

        # r=(0,0,rz) 时：tau_x = -rz*Fy, tau_y = rz*Fx
        fx_from_tau = -self.config.tilt_torque_xy_gain * torque_vector_unit[1] / rz
        fy_from_tau = self.config.tilt_torque_xy_gain * torque_vector_unit[0] / rz
        return (
            thrust_vector_unit[0] + fx_from_tau,
            thrust_vector_unit[1] + fy_from_tau,
            thrust_vector_unit[2],
        )


def _mat_vec_mul(
    mat: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vec: Vec3,
) -> Vec3:
    return (
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    )


def _roll_to_servo_x(
    roll_rad: float,
    servo_limit: float,
    vector_tilt_limit_rad: float,
    roll_sign: float,
    mech_gain: float,
) -> float:
    if vector_tilt_limit_rad <= 1.0e-9:
        return 0.0
    servo_x_linear = roll_sign * mech_gain * roll_rad / vector_tilt_limit_rad
    return clamp(servo_x_linear, -servo_limit, servo_limit)


def _pitch_to_servo_y(
    pitch_rad: float,
    servo_limit: float,
    vector_tilt_limit_rad: float,
    pitch_sign: float,
    mech_gain: float,
) -> float:
    if vector_tilt_limit_rad <= 1.0e-9:
        return 0.0
    servo_y_linear = pitch_sign * mech_gain * pitch_rad / vector_tilt_limit_rad
    return clamp(servo_y_linear, -servo_limit, servo_limit)


def _vector_unit_torque_to_motor_axis(
    torque_vector_unit: Vec3,
    coeff: tuple[float, float, float],
) -> float:
    # TODO: 按真实机构填写系数，当前采用可配置线性投影占位。
    return (
        coeff[0] * torque_vector_unit[0]
        + coeff[1] * torque_vector_unit[1]
        + coeff[2] * torque_vector_unit[2]
    )


def _motor_axis_to_vector_unit_torque(
    motor_axis_torque: float,
    coeff: tuple[float, float, float],
) -> Vec3:
    # TODO: 按真实机构填写系数，当前采用可配置线性回映射占位。
    return (
        coeff[0] * motor_axis_torque,
        coeff[1] * motor_axis_torque,
        coeff[2] * motor_axis_torque,
    )


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
