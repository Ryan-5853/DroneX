from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

if __package__ is None or __package__ == "":
    # Support direct execution: python sim_py/visualization.py
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

from sim_py.calibration import apply_baseline_calibration
from sim_py.config import SimulationConfig
from sim_py.data_types import Quat, Vec3
from sim_py.pipeline import SimulationSnapshot, SimulationSystem


def _quat_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def _quat_mul(a: Quat, b: Quat) -> Quat:
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def _quat_normalize(q: Quat) -> Quat:
    n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    if n2 <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    inv = n2 ** -0.5
    return (q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv)


def _quat_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quat:
    # ZYX (yaw-pitch-roll) intrinsic sequence.
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr = math.cos(0.5 * r)
    sr = math.sin(0.5 * r)
    cp = math.cos(0.5 * p)
    sp = math.sin(0.5 * p)
    cy = math.cos(0.5 * y)
    sy = math.sin(0.5 * y)

    q_body_to_world: Quat = (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    )
    # Internal convention is world -> body.
    return _quat_conj(_quat_normalize(q_body_to_world))


def _quat_to_rot_body_to_world(q_world_to_body: Quat) -> np.ndarray:
    # Convert world->body quaternion to body->world rotation matrix.
    q = _quat_conj(q_world_to_body)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _quat_error_vec(q_des_world_to_body: Quat, q_est_world_to_body: Quat) -> Vec3:
    q_err = _quat_mul(q_des_world_to_body, _quat_conj(q_est_world_to_body))
    if q_err[0] < 0.0:
        q_err = (-q_err[0], -q_err[1], -q_err[2], -q_err[3])
    return (2.0 * q_err[1], 2.0 * q_err[2], 2.0 * q_err[3])


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


@dataclass(slots=True)
class _History:
    t: list[float]
    z: list[float]
    vz: list[float]
    thrust: list[float]
    torque: list[float]


class InteractiveVisualizer:
    """Interactive realtime visualizer for the DroneX Python simulation."""

    def __init__(self, gui: bool = True, full_ui: bool = False) -> None:
        self.gui = gui
        self.full_ui = full_ui
        self.config = SimulationConfig()
        apply_baseline_calibration(self.config)
        self.system = SimulationSystem(self.config)

        self.dt = self.config.dynamics.dt
        self.time = 0.0
        self.paused = False

        self._guidance_disp_target = (0.0, 0.0, 0.0)
        self._guidance_att_gain = 1.8

        self._history = _History(t=[], z=[], vz=[], thrust=[], torque=[])
        self._history_len = 350

        # Rendering settings: keep simulation stable while reducing GUI load.
        self.render_fps = 30.0
        self.sim_steps_per_frame = max(1, int(round((1.0 / self.render_fps) / self.dt)))
        self._plot_update_stride = 2
        self._text_update_stride = 3
        self._frame_counter = 0
        self._last_snapshot: SimulationSnapshot | None = None

        if self.gui:
            self._build_figure()
            self._build_geometry()
            self._build_controls()
            self._animation = FuncAnimation(
                self.fig,
                self._tick,
                interval=int(1000.0 / self.render_fps),
                blit=False,
                cache_frame_data=False,
            )

    def _build_figure(self) -> None:
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.40, wspace=0.25, hspace=0.32)

        self.ax3d = self.fig.add_subplot(2, 2, 1, projection="3d")
        self.ax3d.set_title("3D Flight View")
        self.ax3d.set_xlim(-6.0, 6.0)
        self.ax3d.set_ylim(-6.0, 6.0)
        self.ax3d.set_zlim(-1.0, 10.0)
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")

        # Ground grid
        gx = np.linspace(-6.0, 6.0, 13)
        gy = np.linspace(-6.0, 6.0, 13)
        for x in gx:
            self.ax3d.plot([x, x], [gy[0], gy[-1]], [0.0, 0.0], color="#d0d0d0", linewidth=0.7)
        for y in gy:
            self.ax3d.plot([gx[0], gx[-1]], [y, y], [0.0, 0.0], color="#d0d0d0", linewidth=0.7)

        self.ax_state = self.fig.add_subplot(2, 2, 2)
        self.ax_state.set_title("Altitude / Vertical Speed")
        self.ax_state.set_xlabel("Time [s]")
        self.ax_state.set_ylabel("Value")
        self.line_z, = self.ax_state.plot([], [], label="z [m]", color="#0b84a5", linewidth=2.0)
        self.line_vz, = self.ax_state.plot([], [], label="vz [m/s]", color="#f6c85f", linewidth=1.8)
        self.ax_state.legend(loc="upper right")

        self.ax_signal = self.fig.add_subplot(2, 2, 4)
        self.ax_signal.set_title("Controller Outputs")
        self.ax_signal.set_xlabel("Time [s]")
        self.ax_signal.set_ylabel("Norm")
        self.line_thrust, = self.ax_signal.plot([], [], label="|thrust_cmd|", color="#6f4e7c", linewidth=2.0)
        self.line_torque, = self.ax_signal.plot([], [], label="|torque_cmd|", color="#9dd866", linewidth=2.0)
        self.ax_signal.legend(loc="upper right")

        self.ax_text = self.fig.add_subplot(2, 2, 3)
        self.ax_text.set_title("Realtime Telemetry")
        self.ax_text.axis("off")
        self.text_block = self.ax_text.text(
            0.01,
            0.98,
            "",
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
        )

    def _build_geometry(self) -> None:
        # Rocket body in body frame.
        self.body_radius = 0.22
        self.body_height = 1.8
        self.vec_unit_offset = np.array([0.0, 0.0, -0.95], dtype=float)
        theta = np.linspace(0.0, 2.0 * math.pi, 24)

        z_top = 0.5 * self.body_height
        z_bot = -0.5 * self.body_height
        self.body_top = np.vstack((self.body_radius * np.cos(theta), self.body_radius * np.sin(theta), np.full_like(theta, z_top))).T
        self.body_bot = np.vstack((self.body_radius * np.cos(theta), self.body_radius * np.sin(theta), np.full_like(theta, z_bot))).T

        self.body_top_line, = self.ax3d.plot([], [], [], color="#1f77b4", linewidth=2.0)
        self.body_bot_line, = self.ax3d.plot([], [], [], color="#1f77b4", linewidth=2.0)

        self.body_vertical_lines = []
        for idx in range(0, len(theta), 4):
            line, = self.ax3d.plot([], [], [], color="#1f77b4", linewidth=1.2)
            self.body_vertical_lines.append((idx, line))

        # Vectoring unit geometry in vector-unit frame.
        self.vu_radius = 0.3
        self.vu_ring = np.vstack((self.vu_radius * np.cos(theta), self.vu_radius * np.sin(theta), np.zeros_like(theta))).T
        self.vu_x_arm = np.array([[-self.vu_radius, 0.0, 0.0], [self.vu_radius, 0.0, 0.0]], dtype=float)
        self.vu_y_arm = np.array([[0.0, -self.vu_radius, 0.0], [0.0, self.vu_radius, 0.0]], dtype=float)

        self.vu_ring_line, = self.ax3d.plot([], [], [], color="#d95f02", linewidth=2.0)
        self.vu_x_arm_line, = self.ax3d.plot([], [], [], color="#d95f02", linewidth=2.4)
        self.vu_y_arm_line, = self.ax3d.plot([], [], [], color="#d95f02", linewidth=2.4)

        self.path_line, = self.ax3d.plot([], [], [], color="#2ca02c", linewidth=1.4)
        self.path_xyz: list[Vec3] = []

        self.thrust_vector_line, = self.ax3d.plot([], [], [], color="#e7298a", linewidth=2.0)

    def _build_controls(self) -> None:
        self.sliders: dict[str, Slider] = {}

        guidance_specs = [
            ("roll", "Desired Roll [deg]", -45.0, 45.0, 0.0),
            ("pitch", "Desired Pitch [deg]", -45.0, 45.0, 0.0),
            ("yaw", "Desired Yaw [deg]", -180.0, 180.0, 0.0),
            ("vx", "Desired Vx [m/s]", -4.0, 4.0, 0.0),
            ("vy", "Desired Vy [m/s]", -4.0, 4.0, 0.0),
            ("vz", "Desired Vz [m/s]", -3.0, 3.0, 0.0),
            ("wind_speed", "Wind Speed [m/s]", 0.0, 18.0, 0.0),
            ("wind_dir", "Wind Dir [deg]", -180.0, 180.0, 0.0),
            ("wind_up", "Wind Up [m/s]", -6.0, 6.0, 0.0),
            ("gust", "Wind Gust Std", 0.0, 4.0, 0.0),
        ]

        if self.full_ui:
            guidance_specs.extend(
                [
                    ("dfx", "Disturb Fx [N]", -12.0, 12.0, 0.0),
                    ("dfy", "Disturb Fy [N]", -12.0, 12.0, 0.0),
                    ("dfz", "Disturb Fz [N]", -12.0, 12.0, 0.0),
                ]
            )

        tuning_specs = [
            ("pos_kp_xy", "Ctrl Pos Kp XY", 0.0, 4.0, self.config.controller.pos_kp_xyz[0]),
            ("pos_kp_z", "Ctrl Pos Kp Z", 0.0, 6.0, self.config.controller.pos_kp_xyz[2]),
            ("vel_kp_xy", "Ctrl Vel Kd XY", 0.0, 4.0, self.config.controller.vel_kp_xyz[0]),
            ("vel_kp_z", "Ctrl Vel Kd Z", 0.0, 6.0, self.config.controller.vel_kp_xyz[2]),
            ("pos_ki_z", "Ctrl Pos Ki Z", 0.0, 1.0, self.config.controller.pos_ki_xyz[2]),
            ("att_kp_xy", "Ctrl Att Kp XY", 0.0, 10.0, self.config.controller.att_kp_xyz[0]),
            ("att_kp_z", "Ctrl Att Kp Z", 0.0, 10.0, self.config.controller.att_kp_xyz[2]),
            ("rate_kp_xy", "Ctrl Rate Kd XY", 0.0, 2.0, self.config.controller.rate_kp_xyz[0]),
            ("rate_kp_z", "Ctrl Rate Kd Z", 0.0, 2.0, self.config.controller.rate_kp_xyz[2]),
            ("att_ki_xy", "Ctrl Att Ki XY", 0.0, 1.0, self.config.controller.att_ki_xyz[0]),
            ("thrust_max", "Ctrl Thrust Max", 5.0, 60.0, self.config.controller.thrust_max),
            ("torque_xy_lim", "Ctrl Torque Lim XY", 0.5, 12.0, self.config.controller.torque_limit_xyz[0]),
            ("torque_z_lim", "Ctrl Torque Lim Z", 0.2, 8.0, self.config.controller.torque_limit_xyz[2]),
            ("thrust2cmd", "Alloc Thrust2Cmd", 0.01, 0.30, self.config.allocator.thrust_to_motor_cmd_gain),
            ("motor_gain", "Act Motor Thrust Gain", 1.0, 12.0, self.config.actuator.motor_thrust_gain[0]),
        ]

        y0 = 0.35
        dy = 0.022
        for idx, (key, label, vmin, vmax, vinit) in enumerate(guidance_specs):
            y = y0 - idx * dy
            ax = self.fig.add_axes([0.06, y, 0.42, 0.014])
            slider = Slider(ax=ax, label=label, valmin=vmin, valmax=vmax, valinit=vinit)
            self.sliders[key] = slider

        if self.full_ui:
            for idx, (key, label, vmin, vmax, vinit) in enumerate(tuning_specs):
                y = y0 - idx * dy
                ax = self.fig.add_axes([0.53, y, 0.42, 0.014])
                slider = Slider(ax=ax, label=label, valmin=vmin, valmax=vmax, valinit=vinit)
                self.sliders[key] = slider

        ax_pause = self.fig.add_axes([0.70, 0.02, 0.11, 0.05])
        self.btn_pause = Button(ax_pause, "Pause/Run")
        self.btn_pause.on_clicked(self._on_pause)

        ax_reset = self.fig.add_axes([0.83, 0.02, 0.11, 0.05])
        self.btn_reset = Button(ax_reset, "Reset")
        self.btn_reset.on_clicked(self._on_reset)

    def _on_pause(self, _event: object) -> None:
        self.paused = not self.paused
        if self.gui and hasattr(self, "_animation"):
            if self.paused:
                self._animation.event_source.stop()
            else:
                self._animation.event_source.start()

    def _on_reset(self, _event: object) -> None:
        self.system = SimulationSystem(self.config)
        self.time = 0.0
        self._guidance_disp_target = (0.0, 0.0, 0.0)
        self._history = _History(t=[], z=[], vz=[], thrust=[], torque=[])
        self._frame_counter = 0
        self._last_snapshot = None
        if self.gui:
            self.path_xyz = []

    def _apply_user_inputs(self) -> None:
        if not self.gui:
            return

        roll = self.sliders["roll"].val
        pitch = self.sliders["pitch"].val
        yaw = self.sliders["yaw"].val
        vx = self.sliders["vx"].val
        vy = self.sliders["vy"].val
        vz = self.sliders["vz"].val

        desired_q = _quat_from_euler_deg(roll, pitch, yaw)
        state_q = self.system.state.att_quat
        err_vec = _quat_error_vec(desired_q, state_q)
        self.config.target.att_quat = desired_q
        self.config.target.angv_xyz = (
            self._guidance_att_gain * err_vec[0],
            self._guidance_att_gain * err_vec[1],
            self._guidance_att_gain * err_vec[2],
        )

        self._guidance_disp_target = (
            self._guidance_disp_target[0] + vx * self.dt,
            self._guidance_disp_target[1] + vy * self.dt,
            self._guidance_disp_target[2] + vz * self.dt,
        )
        self.config.target.disp_xyz = self._guidance_disp_target
        self.config.target.vel_xyz = (vx, vy, vz)

        wind_speed = self.sliders["wind_speed"].val
        wind_dir = math.radians(self.sliders["wind_dir"].val)
        wind_up = self.sliders["wind_up"].val
        self.config.dynamics.wind_world_mps = (
            wind_speed * math.cos(wind_dir),
            wind_speed * math.sin(wind_dir),
            wind_up,
        )
        self.config.dynamics.wind_gust_std = self.sliders["gust"].val

        if "dfx" in self.sliders:
            self.config.dynamics.disturbance_force_body = (
                self.sliders["dfx"].val,
                self.sliders["dfy"].val,
                self.sliders["dfz"].val,
            )

        if "pos_kp_xy" in self.sliders:
            # Online controller tuning.
            pos_kp_xy = self.sliders["pos_kp_xy"].val
            pos_kp_z = self.sliders["pos_kp_z"].val
            vel_kp_xy = self.sliders["vel_kp_xy"].val
            vel_kp_z = self.sliders["vel_kp_z"].val
            pos_ki_z = self.sliders["pos_ki_z"].val

            self.config.controller.pos_kp_xyz = (pos_kp_xy, pos_kp_xy, pos_kp_z)
            self.config.controller.vel_kp_xyz = (vel_kp_xy, vel_kp_xy, vel_kp_z)
            self.config.controller.pos_ki_xyz = (0.0, 0.0, pos_ki_z)

            att_kp_xy = self.sliders["att_kp_xy"].val
            att_kp_z = self.sliders["att_kp_z"].val
            rate_kp_xy = self.sliders["rate_kp_xy"].val
            rate_kp_z = self.sliders["rate_kp_z"].val
            att_ki_xy = self.sliders["att_ki_xy"].val

            self.config.controller.att_kp_xyz = (att_kp_xy, att_kp_xy, att_kp_z)
            self.config.controller.rate_kp_xyz = (rate_kp_xy, rate_kp_xy, rate_kp_z)
            self.config.controller.att_ki_xyz = (att_ki_xy, att_ki_xy, 0.0)

            self.config.controller.thrust_max = self.sliders["thrust_max"].val
            self.config.controller.torque_limit_xyz = (
                self.sliders["torque_xy_lim"].val,
                self.sliders["torque_xy_lim"].val,
                self.sliders["torque_z_lim"].val,
            )

            # Online allocation/actuation tuning.
            self.config.allocator.thrust_to_motor_cmd_gain = self.sliders["thrust2cmd"].val
            motor_gain = self.sliders["motor_gain"].val
            self.config.actuator.motor_thrust_gain = (motor_gain, motor_gain)

    def _append_history(self, snapshot: SimulationSnapshot) -> None:
        self._history.t.append(self.time)
        self._history.z.append(snapshot.pos_truth.disp_xyz[2])
        self._history.vz.append(snapshot.pos_truth.vel_xyz[2])
        self._history.thrust.append(_norm(snapshot.ctrl_output.thrust_xyz))
        self._history.torque.append(_norm(snapshot.ctrl_output.torque_xyz))

        if len(self._history.t) > self._history_len:
            self._history.t.pop(0)
            self._history.z.pop(0)
            self._history.vz.pop(0)
            self._history.thrust.pop(0)
            self._history.torque.pop(0)

    def _update_2d_plots(self) -> None:
        if not self._history.t:
            return

        self.line_z.set_data(self._history.t, self._history.z)
        self.line_vz.set_data(self._history.t, self._history.vz)
        self.line_thrust.set_data(self._history.t, self._history.thrust)
        self.line_torque.set_data(self._history.t, self._history.torque)

        t0 = self._history.t[0]
        t1 = self._history.t[-1] + 1.0e-6
        self.ax_state.set_xlim(t0, t1)
        self.ax_signal.set_xlim(t0, t1)

        if self._frame_counter % self._plot_update_stride == 0:
            self.ax_state.relim()
            self.ax_state.autoscale_view(scalex=False, scaley=True)
            self.ax_signal.relim()
            self.ax_signal.autoscale_view(scalex=False, scaley=True)

    def _update_3d_view(self, snapshot: SimulationSnapshot) -> None:
        pos = np.array(snapshot.pos_truth.disp_xyz, dtype=float)
        rot_bw = _quat_to_rot_body_to_world(snapshot.pos_truth.att_quat)

        top_w = (rot_bw @ self.body_top.T).T + pos
        bot_w = (rot_bw @ self.body_bot.T).T + pos
        self.body_top_line.set_data(top_w[:, 0], top_w[:, 1])
        self.body_top_line.set_3d_properties(top_w[:, 2])
        self.body_bot_line.set_data(bot_w[:, 0], bot_w[:, 1])
        self.body_bot_line.set_3d_properties(bot_w[:, 2])

        for idx, line in self.body_vertical_lines:
            p1 = top_w[idx]
            p2 = bot_w[idx]
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            line.set_3d_properties([p1[2], p2[2]])

        servo_x = snapshot.ctrl_cmd.servo_x
        servo_y = snapshot.ctrl_cmd.servo_y
        roll = self.config.actuator.servo_angle_offset_rad[0] + self.config.actuator.servo_angle_gain_rad[0] * servo_x
        pitch = self.config.actuator.servo_angle_offset_rad[1] + self.config.actuator.servo_angle_gain_rad[1] * servo_y

        rot_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(pitch), -math.sin(pitch)],
                [0.0, math.sin(pitch), math.cos(pitch)],
            ],
            dtype=float,
        )
        rot_y = np.array(
            [
                [math.cos(roll), 0.0, math.sin(roll)],
                [0.0, 1.0, 0.0],
                [-math.sin(roll), 0.0, math.cos(roll)],
            ],
            dtype=float,
        )
        rot_vu_to_body = rot_y @ rot_x

        origin_body = self.vec_unit_offset
        ring_body = (rot_vu_to_body @ self.vu_ring.T).T + origin_body
        x_arm_body = (rot_vu_to_body @ self.vu_x_arm.T).T + origin_body
        y_arm_body = (rot_vu_to_body @ self.vu_y_arm.T).T + origin_body

        ring_world = (rot_bw @ ring_body.T).T + pos
        x_arm_world = (rot_bw @ x_arm_body.T).T + pos
        y_arm_world = (rot_bw @ y_arm_body.T).T + pos

        self.vu_ring_line.set_data(ring_world[:, 0], ring_world[:, 1])
        self.vu_ring_line.set_3d_properties(ring_world[:, 2])
        self.vu_x_arm_line.set_data(x_arm_world[:, 0], x_arm_world[:, 1])
        self.vu_x_arm_line.set_3d_properties(x_arm_world[:, 2])
        self.vu_y_arm_line.set_data(y_arm_world[:, 0], y_arm_world[:, 1])
        self.vu_y_arm_line.set_3d_properties(y_arm_world[:, 2])

        self.path_xyz.append((pos[0], pos[1], pos[2]))
        if len(self.path_xyz) > self._history_len:
            self.path_xyz.pop(0)
        px = [p[0] for p in self.path_xyz]
        py = [p[1] for p in self.path_xyz]
        pz = [p[2] for p in self.path_xyz]
        self.path_line.set_data(px, py)
        self.path_line.set_3d_properties(pz)

        origin_world = (rot_bw @ origin_body) + pos
        force_world = rot_bw @ np.array(snapshot.actor_output.force_xyz, dtype=float)
        force_vis = force_world * 0.2
        tip_world = origin_world + force_vis
        self.thrust_vector_line.set_data([origin_world[0], tip_world[0]], [origin_world[1], tip_world[1]])
        self.thrust_vector_line.set_3d_properties([origin_world[2], tip_world[2]])

    def _update_text(self, snapshot: SimulationSnapshot) -> None:
        wind = self.config.dynamics.wind_world_mps
        txt = (
            f"t = {self.time:7.2f}s\n"
            f"pos_truth xyz = ({snapshot.pos_truth.disp_xyz[0]:7.3f}, {snapshot.pos_truth.disp_xyz[1]:7.3f}, {snapshot.pos_truth.disp_xyz[2]:7.3f})\n"
            f"vel_truth xyz = ({snapshot.pos_truth.vel_xyz[0]:7.3f}, {snapshot.pos_truth.vel_xyz[1]:7.3f}, {snapshot.pos_truth.vel_xyz[2]:7.3f})\n"
            f"att_quat wxyz = ({snapshot.pos_truth.att_quat[0]:7.4f}, {snapshot.pos_truth.att_quat[1]:7.4f}, {snapshot.pos_truth.att_quat[2]:7.4f}, {snapshot.pos_truth.att_quat[3]:7.4f})\n"
            f"\n"
            f"sensor gyro = ({snapshot.sensor_obs.angv_xyz[0]:7.3f}, {snapshot.sensor_obs.angv_xyz[1]:7.3f}, {snapshot.sensor_obs.angv_xyz[2]:7.3f})\n"
            f"sensor acc  = ({snapshot.sensor_obs.acc_xyz[0]:7.3f}, {snapshot.sensor_obs.acc_xyz[1]:7.3f}, {snapshot.sensor_obs.acc_xyz[2]:7.3f})\n"
            f"\n"
            f"ctrl thrust = ({snapshot.ctrl_output.thrust_xyz[0]:7.3f}, {snapshot.ctrl_output.thrust_xyz[1]:7.3f}, {snapshot.ctrl_output.thrust_xyz[2]:7.3f})\n"
            f"ctrl torque = ({snapshot.ctrl_output.torque_xyz[0]:7.3f}, {snapshot.ctrl_output.torque_xyz[1]:7.3f}, {snapshot.ctrl_output.torque_xyz[2]:7.3f})\n"
            f"ctrl cmd    = servo({snapshot.ctrl_cmd.servo_x:6.3f}, {snapshot.ctrl_cmd.servo_y:6.3f})  motor({snapshot.ctrl_cmd.motor_p:6.3f}, {snapshot.ctrl_cmd.motor_n:6.3f})\n"
            f"\n"
            f"actor force = ({snapshot.actor_output.force_xyz[0]:7.3f}, {snapshot.actor_output.force_xyz[1]:7.3f}, {snapshot.actor_output.force_xyz[2]:7.3f})\n"
            f"actor torque= ({snapshot.actor_output.torque_xyz[0]:7.3f}, {snapshot.actor_output.torque_xyz[1]:7.3f}, {snapshot.actor_output.torque_xyz[2]:7.3f})\n"
            f"\n"
            f"wind world  = ({wind[0]:6.2f}, {wind[1]:6.2f}, {wind[2]:6.2f})\n"
            f"ctrl gains  = posKpXY {self.config.controller.pos_kp_xyz[0]:5.2f}  posKpZ {self.config.controller.pos_kp_xyz[2]:5.2f}\n"
            f"              attKpXY {self.config.controller.att_kp_xyz[0]:5.2f}  attKpZ {self.config.controller.att_kp_xyz[2]:5.2f}\n"
            f"map gains   = thrust2cmd {self.config.allocator.thrust_to_motor_cmd_gain:5.3f}  motorThrust {self.config.actuator.motor_thrust_gain[0]:5.2f}\n"
        )
        self.text_block.set_text(txt)

    def _tick(self, _frame_idx: int) -> list[object]:
        if self.paused:
            return []

        self._apply_user_inputs()
        snapshot = None
        for _ in range(self.sim_steps_per_frame):
            snapshot = self.system.step()
            self.time += self.dt

        if snapshot is None:
            return []

        self._last_snapshot = snapshot
        self._append_history(snapshot)
        self._frame_counter += 1

        self._update_2d_plots()
        self._update_3d_view(snapshot)
        if self._frame_counter % self._text_update_stride == 0:
            self._update_text(snapshot)

        return []

    def run(self) -> None:
        plt.show()

    def run_headless(self, steps: int) -> None:
        for _ in range(steps):
            self._apply_user_inputs()
            snapshot = self.system.step()
            self.time += self.dt
            self._append_history(snapshot)

        print("Headless run finished")
        print(f"time={self.time:.3f}s")
        print(f"pos={snapshot.pos_truth.disp_xyz}")
        print(f"vel={snapshot.pos_truth.vel_xyz}")
        print(f"att={snapshot.pos_truth.att_quat}")


def main() -> None:
    parser = argparse.ArgumentParser(description="DroneX interactive simulator visualizer")
    parser.add_argument(
        "--headless-steps",
        type=int,
        default=0,
        help="Run without GUI for N steps for smoke test",
    )
    parser.add_argument(
        "--full-ui",
        action="store_true",
        help="Enable full slider set (heavier). Default is compact UI for better responsiveness.",
    )
    args = parser.parse_args()

    app = InteractiveVisualizer(gui=args.headless_steps <= 0, full_ui=args.full_ui)
    if args.headless_steps > 0:
        app.run_headless(args.headless_steps)
        return
    app.run()


if __name__ == "__main__":
    main()
