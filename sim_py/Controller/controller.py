from __future__ import annotations

import math

from sim_py.config import ControllerConfig
from sim_py.data_types import CtrlOutput, POSEst, Quat, SimulationTarget, Vec3
from sim_py.math_utils import clamp


class ControllerModule:
    """将估计位姿与目标位姿比较，生成推力与力矩目标。"""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self._pos_integral = (0.0, 0.0, 0.0)
        self._att_integral = (0.0, 0.0, 0.0)

    def reset(self) -> None:
        self._pos_integral = (0.0, 0.0, 0.0)
        self._att_integral = (0.0, 0.0, 0.0)

    def update(self, pos_est: POSEst, target: SimulationTarget) -> CtrlOutput:
        dt = max(self.config.dt, 1.0e-6)

        thrust_body = (0.0, 0.0, 0.0)
        if self.config.enable_position_loop:
            pos_error = _sub_vec3(target.disp_xyz, pos_est.disp_xyz)
            vel_error = _sub_vec3(target.vel_xyz, pos_est.vel_xyz)

            self._pos_integral = _integrate_with_limit(
                self._pos_integral,
                pos_error,
                dt,
                self.config.pos_integral_limit_xyz,
            )

            acc_cmd_world = _add_vec3(
                _add_vec3(
                    _elem_mul(self.config.pos_kp_xyz, pos_error),
                    _elem_mul(self.config.vel_kp_xyz, vel_error),
                ),
                _elem_mul(self.config.pos_ki_xyz, self._pos_integral),
            )

            thrust_world_unsat = _scale_vec3(
                _sub_vec3(acc_cmd_world, self.config.gravity_world),
                max(self.config.mass_estimate, 1.0e-6),
            )
            thrust_world = _saturate_norm(thrust_world_unsat, self.config.thrust_min, self.config.thrust_max)

            # Position-loop anti-windup on saturation mismatch.
            aw_pos = _scale_vec3(
                _sub_vec3(thrust_world, thrust_world_unsat),
                self.config.anti_windup_gain / max(self.config.mass_estimate, 1.0e-6),
            )
            self._pos_integral = _add_vec3(self._pos_integral, _scale_vec3(aw_pos, dt))
            self._pos_integral = _limit_vec3(self._pos_integral, self.config.pos_integral_limit_xyz)

            thrust_body = _rotate_world_to_body(pos_est.att_quat, thrust_world)

        torque_body = (0.0, 0.0, 0.0)
        if self.config.enable_attitude_loop:
            q_err = _quat_mul(target.att_quat, _quat_conj(pos_est.att_quat))
            q_err = _flip_positive_scalar(q_err)
            att_error = (2.0 * q_err[1], 2.0 * q_err[2], 2.0 * q_err[3])
            rate_error = _sub_vec3(target.angv_xyz, pos_est.angv_xyz)

            self._att_integral = _integrate_with_limit(
                self._att_integral,
                att_error,
                dt,
                self.config.att_integral_limit_xyz,
            )

            torque_unsat = _add_vec3(
                _add_vec3(
                    _elem_mul(self.config.att_kp_xyz, att_error),
                    _elem_mul(self.config.rate_kp_xyz, rate_error),
                ),
                _elem_mul(self.config.att_ki_xyz, self._att_integral),
            )
            torque_body = _limit_vec3(torque_unsat, self.config.torque_limit_xyz)

            # Attitude-loop anti-windup on saturation mismatch.
            aw_att = _scale_vec3(_sub_vec3(torque_body, torque_unsat), self.config.anti_windup_gain)
            self._att_integral = _add_vec3(self._att_integral, _scale_vec3(aw_att, dt))
            self._att_integral = _limit_vec3(self._att_integral, self.config.att_integral_limit_xyz)

        return CtrlOutput(
            thrust_xyz=thrust_body,
            torque_xyz=torque_body,
        )


def _elem_mul(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] * b[0], a[1] * b[1], a[2] * b[2])


def _add_vec3(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub_vec3(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale_vec3(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _limit_vec3(v: Vec3, limit: Vec3) -> Vec3:
    return (
        clamp(v[0], -abs(limit[0]), abs(limit[0])),
        clamp(v[1], -abs(limit[1]), abs(limit[1])),
        clamp(v[2], -abs(limit[2]), abs(limit[2])),
    )


def _integrate_with_limit(integral: Vec3, signal: Vec3, dt: float, limit: Vec3) -> Vec3:
    return _limit_vec3(
        (
            integral[0] + signal[0] * dt,
            integral[1] + signal[1] * dt,
            integral[2] + signal[2] * dt,
        ),
        limit,
    )


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _saturate_norm(v: Vec3, min_norm: float, max_norm: float) -> Vec3:
    n = _norm(v)
    if n <= 1.0e-9:
        if min_norm <= 0.0:
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, min_norm)

    clamped = clamp(n, max(min_norm, 0.0), max(max_norm, max(min_norm, 0.0)))
    s = clamped / n
    return (v[0] * s, v[1] * s, v[2] * s)


def _rotate_world_to_body(q_wb: Quat, vec_world: Vec3) -> Vec3:
    vq: Quat = (0.0, vec_world[0], vec_world[1], vec_world[2])
    out = _quat_mul(_quat_mul(q_wb, vq), _quat_conj(q_wb))
    return (out[1], out[2], out[3])


def _quat_mul(a: Quat, b: Quat) -> Quat:
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def _quat_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def _flip_positive_scalar(q: Quat) -> Quat:
    if q[0] < 0.0:
        return (-q[0], -q[1], -q[2], -q[3])
    return q
