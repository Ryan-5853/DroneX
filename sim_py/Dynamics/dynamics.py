from __future__ import annotations

import math
import random

from sim_py.config import DynamicsConfig
from sim_py.data_types import ActorOutput, POSTruth, Quat, Vec3
from sim_py.math_utils import add_vec3, normalize_quat, scale_vec3, sub_vec3


class DynamicsModule:
    """根据执行器实际输出推进真实物理状态。"""

    def __init__(self, config: DynamicsConfig, initial_state: POSTruth) -> None:
        self.config = config
        self.state = initial_state

    def step(self, actor_output: ActorOutput) -> POSTruth:
        dt = self.config.dt

        mass = max(self.config.mass, 1.0e-6)
        q_wb = self.state.att_quat
        omega_b = self.state.angv_xyz

        force_body_total = self._compose_body_force(actor_output.force_xyz)
        force_world_from_body = _rotate_body_to_world(q_wb, force_body_total)

        wind_world = self._sample_wind_world()
        rel_air_vel_world = sub_vec3(self.state.vel_xyz, wind_world)
        drag_world = self._compute_aero_drag_world(rel_air_vel_world)

        total_force_world = add_vec3(
            add_vec3(force_world_from_body, drag_world),
            scale_vec3(self.config.gravity_world, mass),
        )
        acc_world = scale_vec3(total_force_world, 1.0 / mass)
        vel_world = add_vec3(self.state.vel_xyz, scale_vec3(acc_world, dt))
        disp_world = add_vec3(self.state.disp_xyz, scale_vec3(vel_world, dt))

        torque_body_total = self._compose_body_torque(actor_output.torque_xyz, q_wb, mass)
        omega_dot_body = self._compute_angular_accel_body(omega_b, torque_body_total)
        angv_body = add_vec3(omega_b, scale_vec3(omega_dot_body, dt))
        att_quat = _integrate_quat_world_to_body(q_wb, angv_body, dt)

        self.state = POSTruth(
            disp_xyz=disp_world,
            att_quat=att_quat,
            vel_xyz=vel_world,
            acc_xyz=acc_world,
            angv_xyz=angv_body,
            anga_xyz=omega_dot_body,
        )
        return self.state

    def _compose_body_force(self, actuator_force_body: Vec3) -> Vec3:
        disturbance = self.config.disturbance_force_body
        if self.config.disturbance_force_noise_std > 0.0:
            disturbance = (
                disturbance[0] + random.gauss(0.0, self.config.disturbance_force_noise_std),
                disturbance[1] + random.gauss(0.0, self.config.disturbance_force_noise_std),
                disturbance[2] + random.gauss(0.0, self.config.disturbance_force_noise_std),
            )
        return add_vec3(actuator_force_body, disturbance)

    def _compose_body_torque(self, actuator_torque_body: Vec3, q_wb: Quat, mass: float) -> Vec3:
        disturbance = self.config.disturbance_torque_body
        if self.config.disturbance_torque_noise_std > 0.0:
            disturbance = (
                disturbance[0] + random.gauss(0.0, self.config.disturbance_torque_noise_std),
                disturbance[1] + random.gauss(0.0, self.config.disturbance_torque_noise_std),
                disturbance[2] + random.gauss(0.0, self.config.disturbance_torque_noise_std),
            )
        damping = self._compute_angular_damping(self.state.angv_xyz)
        gravity_lever_torque = (0.0, 0.0, 0.0)
        if self.config.enable_gravity_lever_torque:
            gravity_force_body = _rotate_world_to_body(q_wb, scale_vec3(self.config.gravity_world, mass))
            r_thrust_to_com = sub_vec3(self.config.center_of_mass_body_m, self.config.thrust_point_body_m)
            gravity_lever_torque = scale_vec3(
                _cross(r_thrust_to_com, gravity_force_body),
                self.config.gravity_lever_torque_gain,
            )

        return add_vec3(
            add_vec3(actuator_torque_body, disturbance),
            add_vec3(damping, gravity_lever_torque),
        )

    def _sample_wind_world(self) -> Vec3:
        base = self.config.wind_world_mps
        gust_std = self.config.wind_gust_std
        if gust_std <= 0.0:
            return base
        return (
            base[0] + random.gauss(0.0, gust_std),
            base[1] + random.gauss(0.0, gust_std),
            base[2] + random.gauss(0.0, gust_std),
        )

    def _compute_aero_drag_world(self, rel_air_vel_world: Vec3) -> Vec3:
        lin = self.config.linear_drag_coeff_world
        linear_drag = (
            -lin[0] * rel_air_vel_world[0],
            -lin[1] * rel_air_vel_world[1],
            -lin[2] * rel_air_vel_world[2],
        )
        speed = _norm(rel_air_vel_world)
        if self.config.quadratic_drag_coeff <= 0.0 or speed <= 1.0e-9:
            return linear_drag
        quad_scale = -self.config.quadratic_drag_coeff * speed
        quad_drag = scale_vec3(rel_air_vel_world, quad_scale)
        return add_vec3(linear_drag, quad_drag)

    def _compute_angular_damping(self, omega_b: Vec3) -> Vec3:
        lin = self.config.angular_damping_linear
        damping_lin = (
            -lin[0] * omega_b[0],
            -lin[1] * omega_b[1],
            -lin[2] * omega_b[2],
        )
        mag = _norm(omega_b)
        if self.config.angular_damping_quadratic <= 0.0 or mag <= 1.0e-9:
            return damping_lin
        damping_quad = scale_vec3(omega_b, -self.config.angular_damping_quadratic * mag)
        return add_vec3(damping_lin, damping_quad)

    def _compute_angular_accel_body(self, omega_b: Vec3, torque_b: Vec3) -> Vec3:
        ix, iy, iz = self.config.inertia_body_diag
        ix = max(ix, 1.0e-6)
        iy = max(iy, 1.0e-6)
        iz = max(iz, 1.0e-6)

        i_omega = (ix * omega_b[0], iy * omega_b[1], iz * omega_b[2])
        gyro = _cross(omega_b, i_omega)
        net = sub_vec3(torque_b, gyro)
        return (net[0] / ix, net[1] / iy, net[2] / iz)


def _integrate_quat_world_to_body(q_wb: Quat, omega_body: Vec3, dt: float) -> Quat:
    omega_q: Quat = (0.0, omega_body[0], omega_body[1], omega_body[2])
    q_dot = _scale_quat(_quat_mul(q_wb, omega_q), 0.5)
    q_next = (
        q_wb[0] + q_dot[0] * dt,
        q_wb[1] + q_dot[1] * dt,
        q_wb[2] + q_dot[2] * dt,
        q_wb[3] + q_dot[3] * dt,
    )
    return normalize_quat(q_next)


def _rotate_body_to_world(q_wb: Quat, v_body: Vec3) -> Vec3:
    vq: Quat = (0.0, v_body[0], v_body[1], v_body[2])
    q_bw = _quat_conj(q_wb)
    out = _quat_mul(_quat_mul(q_bw, vq), q_wb)
    return (out[1], out[2], out[3])


def _rotate_world_to_body(q_wb: Quat, v_world: Vec3) -> Vec3:
    vq: Quat = (0.0, v_world[0], v_world[1], v_world[2])
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


def _scale_quat(q: Quat, s: float) -> Quat:
    return (q[0] * s, q[1] * s, q[2] * s, q[3] * s)


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
