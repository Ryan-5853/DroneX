from __future__ import annotations

import math
import random

from sim_py.config import ActuatorConfig
from sim_py.data_types import ActorOutput, CtrlCmd, Vec3
from sim_py.math_utils import clamp


class ActuatorModule:
    """根据控制指令生成带执行器特性的实际输出。"""

    def __init__(self, config: ActuatorConfig) -> None:
        self.config = config
        self._servo_state = [0.0, 0.0]
        self._motor_state = [0.0, 0.0]
        self._servo_bias = [0.0, 0.0]
        self._motor_bias = [0.0, 0.0]
        self._prev_servo_angle_rad = [0.0, 0.0]

    def apply(self, ctrl_cmd: CtrlCmd) -> ActorOutput:
        dt = max(self.config.dt, 1.0e-6)

        self._update_random_walk_bias(dt)

        servo_cmd = [
            clamp(ctrl_cmd.servo_x, -self.config.servo_cmd_limit, self.config.servo_cmd_limit),
            clamp(ctrl_cmd.servo_y, -self.config.servo_cmd_limit, self.config.servo_cmd_limit),
        ]
        motor_cmd = [
            clamp(ctrl_cmd.motor_p, 0.0, self.config.motor_cmd_limit),
            clamp(ctrl_cmd.motor_n, 0.0, self.config.motor_cmd_limit),
        ]

        servo_target = [
            servo_cmd[i] + self._servo_bias[i] + random.gauss(0.0, self.config.servo_cmd_noise_std)
            for i in range(2)
        ]
        motor_target = [
            motor_cmd[i] + self._motor_bias[i] + random.gauss(0.0, self.config.motor_cmd_noise_std)
            for i in range(2)
        ]

        servo_target[0] = clamp(servo_target[0], -self.config.servo_cmd_limit, self.config.servo_cmd_limit)
        servo_target[1] = clamp(servo_target[1], -self.config.servo_cmd_limit, self.config.servo_cmd_limit)
        motor_target[0] = clamp(motor_target[0], 0.0, self.config.motor_cmd_limit)
        motor_target[1] = clamp(motor_target[1], 0.0, self.config.motor_cmd_limit)

        self._servo_state[0] = _step_with_lag_and_rate_limit(
            current=self._servo_state[0],
            target=servo_target[0],
            dt=dt,
            tau=self._effective_tau(self.config.servo_time_constant),
            rate_limit=self.config.servo_rate_limit,
        )
        self._servo_state[1] = _step_with_lag_and_rate_limit(
            current=self._servo_state[1],
            target=servo_target[1],
            dt=dt,
            tau=self._effective_tau(self.config.servo_time_constant),
            rate_limit=self.config.servo_rate_limit,
        )
        self._motor_state[0] = _step_with_lag_and_rate_limit(
            current=self._motor_state[0],
            target=motor_target[0],
            dt=dt,
            tau=self._effective_tau(self.config.motor_time_constant),
            rate_limit=self.config.motor_rate_limit,
        )
        self._motor_state[1] = _step_with_lag_and_rate_limit(
            current=self._motor_state[1],
            target=motor_target[1],
            dt=dt,
            tau=self._effective_tau(self.config.motor_time_constant),
            rate_limit=self.config.motor_rate_limit,
        )

        self._servo_state[0] = clamp(self._servo_state[0], -self.config.servo_cmd_limit, self.config.servo_cmd_limit)
        self._servo_state[1] = clamp(self._servo_state[1], -self.config.servo_cmd_limit, self.config.servo_cmd_limit)
        self._motor_state[0] = clamp(self._motor_state[0], 0.0, self.config.motor_cmd_limit)
        self._motor_state[1] = clamp(self._motor_state[1], 0.0, self.config.motor_cmd_limit)

        servo_angle_x = (
            self.config.servo_angle_offset_rad[0]
            + self.config.servo_angle_gain_rad[0] * self._servo_state[0]
        )
        servo_angle_y = (
            self.config.servo_angle_offset_rad[1]
            + self.config.servo_angle_gain_rad[1] * self._servo_state[1]
        )
        servo_rate = [
            (servo_angle_x - self._prev_servo_angle_rad[0]) / dt,
            (servo_angle_y - self._prev_servo_angle_rad[1]) / dt,
        ]
        self._prev_servo_angle_rad[0] = servo_angle_x
        self._prev_servo_angle_rad[1] = servo_angle_y

        thrust_mag = _motor_cmd_to_thrust(
            self._motor_state,
            self.config.motor_thrust_gain,
            self.config.motor_thrust_exp,
        )
        thrust_dir_vector = _compute_tilted_axis(
            base_axis=self.config.thrust_axis_vector_unit,
            roll_rad=servo_angle_x,
            pitch_rad=servo_angle_y,
        )
        force_vector_unit = _scale_vec3(thrust_dir_vector, thrust_mag)

        motor_axis_torque = (
            self.config.motor_axis_torque_gain[0] * self._motor_state[0]
            - self.config.motor_axis_torque_gain[1] * self._motor_state[1]
        )
        torque_motor_vector = _scale_vec3(_normalize_safe(self.config.motor_axis_vector_unit), motor_axis_torque)
        torque_servo_rate_vector = _mat2x_vec(
            self.config.servo_rate_extra_torque_matrix,
            (servo_rate[0], servo_rate[1]),
        )
        torque_vector_unit = _add_vec3(torque_motor_vector, torque_servo_rate_vector)

        force_body = _mat_vec_mul(self.config.vector_to_body_dcm, force_vector_unit)
        torque_body_from_vector = _mat_vec_mul(self.config.vector_to_body_dcm, torque_vector_unit)
        torque_body_from_offset = _cross(self.config.vector_origin_body_m, force_body)
        torque_body = _add_vec3(torque_body_from_vector, torque_body_from_offset)

        return ActorOutput(
            force_xyz=self._add_noise(force_body, self.config.noise_std_force),
            torque_xyz=self._add_noise(torque_body, self.config.noise_std_torque),
        )

    def _effective_tau(self, primary_tau: float) -> float:
        if primary_tau > 0.0:
            return primary_tau
        return max(self.config.response_lag, 1.0e-6)

    def _update_random_walk_bias(self, dt: float) -> None:
        rw_servo = self.config.servo_random_walk_std
        rw_motor = self.config.motor_random_walk_std
        if rw_servo > 0.0:
            self._servo_bias[0] += random.gauss(0.0, rw_servo * math.sqrt(dt))
            self._servo_bias[1] += random.gauss(0.0, rw_servo * math.sqrt(dt))
        if rw_motor > 0.0:
            self._motor_bias[0] += random.gauss(0.0, rw_motor * math.sqrt(dt))
            self._motor_bias[1] += random.gauss(0.0, rw_motor * math.sqrt(dt))

    @staticmethod
    def _add_noise(value: Vec3, std: float) -> Vec3:
        if std <= 0.0:
            return value
        return tuple(component + random.gauss(0.0, std) for component in value)


def _step_with_lag_and_rate_limit(
    current: float,
    target: float,
    dt: float,
    tau: float,
    rate_limit: float,
) -> float:
    alpha = dt / (tau + dt)
    desired = current + alpha * (target - current)
    if rate_limit > 0.0:
        max_delta = rate_limit * dt
        delta = clamp(desired - current, -max_delta, max_delta)
        return current + delta
    return desired


def _motor_cmd_to_thrust(
    motor_cmd: list[float],
    gains: tuple[float, float],
    exp: float,
) -> float:
    exp_safe = max(exp, 1.0e-6)
    p = gains[0] * (max(0.0, motor_cmd[0]) ** exp_safe)
    n = gains[1] * (max(0.0, motor_cmd[1]) ** exp_safe)
    return p + n


def _compute_tilted_axis(base_axis: Vec3, roll_rad: float, pitch_rad: float) -> Vec3:
    axis = _normalize_safe(base_axis)
    axis_pitch = _rot_x(axis, pitch_rad)
    axis_roll = _rot_y(axis_pitch, roll_rad)
    return _normalize_safe(axis_roll)


def _rot_x(v: Vec3, angle_rad: float) -> Vec3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return (v[0], c * v[1] - s * v[2], s * v[1] + c * v[2])


def _rot_y(v: Vec3, angle_rad: float) -> Vec3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])


def _mat_vec_mul(
    mat: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vec: Vec3,
) -> Vec3:
    return (
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    )


def _mat2x_vec(
    mat: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    vec2: tuple[float, float],
) -> Vec3:
    return (
        mat[0][0] * vec2[0] + mat[0][1] * vec2[1],
        mat[1][0] * vec2[0] + mat[1][1] * vec2[1],
        mat[2][0] * vec2[0] + mat[2][1] * vec2[1],
    )


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add_vec3(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale_vec3(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _normalize_safe(v: Vec3) -> Vec3:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1.0e-9:
        return (0.0, 0.0, 1.0)
    inv_n = 1.0 / n
    return (v[0] * inv_n, v[1] * inv_n, v[2] * inv_n)
