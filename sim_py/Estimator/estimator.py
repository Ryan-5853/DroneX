from __future__ import annotations

import math

from sim_py.config import EstimatorConfig
from sim_py.data_types import POSEst, Quat, SensorObs, Vec3
from sim_py.math_utils import normalize_quat


class EstimatorModule:
    """根据传感器观测反解机体位姿。"""

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self._att_quat: Quat = (1.0, 0.0, 0.0, 0.0)

    def estimate(self, sensor_obs: SensorObs) -> POSEst:
        dt = max(self.config.sample_dt, 1.0e-6)
        gyro = sensor_obs.angv_xyz
        acc = sensor_obs.acc_xyz

        # 基于加速度方向进行重力矢量校正，限制在合理模长区间内，降低机动期误修正。
        corr = (0.0, 0.0, 0.0)
        acc_norm = _norm(acc)
        if self.config.acc_norm_min <= acc_norm <= self.config.acc_norm_max:
            acc_dir = _safe_normalize_vec3(acc)
            g_dir_sensor = _rotate_world_to_sensor(self._att_quat, _safe_normalize_vec3(self.config.gravity_ref_world))
            corr = _cross(g_dir_sensor, acc_dir)

        gyro_corr = _add_vec3(gyro, _scale_vec3(corr, self.config.acc_correction_gain))
        self._att_quat = _integrate_quat_world_to_sensor(self._att_quat, gyro_corr, dt)

        return POSEst(
            att_quat=self._att_quat,
            angv_xyz=sensor_obs.angv_xyz,
            acc_xyz=sensor_obs.acc_xyz,
        )


def _integrate_quat_world_to_sensor(q_ws: Quat, angv_sensor: Vec3, dt: float) -> Quat:
    omega_q: Quat = (0.0, angv_sensor[0], angv_sensor[1], angv_sensor[2])
    q_dot = _scale_quat(_quat_mul(q_ws, omega_q), 0.5)
    q_next = (
        q_ws[0] + q_dot[0] * dt,
        q_ws[1] + q_dot[1] * dt,
        q_ws[2] + q_dot[2] * dt,
        q_ws[3] + q_dot[3] * dt,
    )
    return normalize_quat(q_next)


def _rotate_world_to_sensor(q_ws: Quat, vec_world: Vec3) -> Vec3:
    vq: Quat = (0.0, vec_world[0], vec_world[1], vec_world[2])
    q_conj = _quat_conj(q_ws)
    rotated = _quat_mul(_quat_mul(q_ws, vq), q_conj)
    return (rotated[1], rotated[2], rotated[3])


def _quat_mul(a: Quat, b: Quat) -> Quat:
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def _quat_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def _scale_quat(q: Quat, s: float) -> Quat:
    return (q[0] * s, q[1] * s, q[2] * s, q[3] * s)


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _safe_normalize_vec3(v: Vec3) -> Vec3:
    n = _norm(v)
    if n <= 1.0e-9:
        return (0.0, 0.0, 1.0)
    inv_n = 1.0 / n
    return (v[0] * inv_n, v[1] * inv_n, v[2] * inv_n)


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
