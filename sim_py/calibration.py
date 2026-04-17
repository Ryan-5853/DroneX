from __future__ import annotations

from sim_py.config import SimulationConfig


def apply_baseline_calibration(config: SimulationConfig) -> None:
    """应用首轮可调试基线参数。"""

    # 保持各模块采样周期一致，避免离散时基不一致导致的相位误差。
    config.estimator.sample_dt = config.dynamics.dt  # 估计器离散周期[s]
    config.actuator.dt = config.dynamics.dt  # 执行器离散周期[s]
    config.controller.dt = config.dynamics.dt  # 控制器离散周期[s]

    # Dynamics: 机体动力学与外阻力基线。
    config.dynamics.mass = 1.0  # 机体质量[kg]
    config.dynamics.inertia_body_diag = (0.035, 0.035, 0.06)  # 机体系主惯量(Ix, Iy, Iz)[kg*m^2]
    config.dynamics.thrust_point_body_m = (0.0, 0.0, -0.3)  # 推力作用点相对机体系原点位置[m]
    config.dynamics.enable_gravity_lever_torque = True  # 是否启用“质心高于推力点”的重力杠杆力矩
    config.dynamics.gravity_lever_torque_gain = 1.0  # 重力杠杆力矩增益(建模修正系数)
    config.dynamics.linear_drag_coeff_world = (0.08, 0.08, 0.12)  # 世界系线性阻力系数(N/(m/s))
    config.dynamics.angular_damping_linear = (0.05, 0.05, 0.06)  # 机体系角阻尼系数(N*m/(rad/s))

    # Controller: 级联控制（外环位置/速度 + 内环姿态/角速度）。
    config.controller.mass_estimate = config.dynamics.mass  # 控制器质量估计[kg]，用于重力补偿
    config.controller.gravity_world = config.dynamics.gravity_world  # 控制器采用的重力向量[m/s^2]
    config.controller.enable_position_loop = True  # 是否启用位置外环
    config.controller.enable_attitude_loop = True  # 是否启用姿态内环

    config.controller.pos_kp_xyz = (0.8, 0.8, 1.6)  # 位置环P增益(X,Y,Z)
    config.controller.vel_kp_xyz = (1.2, 1.2, 2.2)  # 速度误差反馈增益(等效D项)
    config.controller.pos_ki_xyz = (0.0, 0.0, 0.08)  # 位置环I增益，当前主要给Z轴抗稳态误差
    config.controller.pos_integral_limit_xyz = (1.5, 1.5, 1.5)  # 位置积分限幅，防止积分饱和

    config.controller.att_kp_xyz = (3.4, 3.4, 2.2)  # 姿态误差P增益(Roll, Pitch, Yaw)
    config.controller.rate_kp_xyz = (0.45, 0.45, 0.35)  # 角速度误差反馈增益
    config.controller.att_ki_xyz = (0.02, 0.02, 0.0)  # 姿态I增益，当前不对Yaw积分
    config.controller.att_integral_limit_xyz = (0.6, 0.6, 0.4)  # 姿态积分限幅

    config.controller.thrust_min = 0.0  # 推力指令最小模长[N]
    config.controller.thrust_max = 15.0  # 推力指令最大模长[N]
    config.controller.torque_limit_xyz = (1.0, 1.0, 0.5)  # 力矩指令限幅(N*m)
    config.controller.anti_windup_gain = 0.25  # 抗积分饱和回注增益

    # Allocator: 控制量到舵机/电机指令映射。
    config.allocator.motor_limit = 1.0  # 电机归一化命令上限
    config.allocator.thrust_point_vector_unit_m = (0.0, 0.0, -0.3)  # 矢量单元系推力作用点位置[m]
    config.allocator.tilt_torque_xy_gain = 1.0  # XY扭矩需求折算到偏转推力的增益
    config.allocator.yaw_motor_torque_ratio = 0.10  # 仅保留一小部分Yaw扭矩给电机差速
    config.allocator.thrust_to_motor_cmd_gain = 0.08  # 推力模长->电机共模命令映射增益
    config.allocator.motor_torque_to_diff_cmd_gain = 0.05  # 轴向扭矩->电机差速命令映射增益

    # Actuator: 执行器动态与推力/扭矩模型。
    config.actuator.motor_cmd_limit = config.allocator.motor_limit  # 电机命令限幅，和分配器一致
    config.actuator.motor_time_constant = 0.06  # 电机一阶响应时间常数[s]
    config.actuator.servo_time_constant = 0.04  # 舵机一阶响应时间常数[s]
    config.actuator.motor_rate_limit = 25.0  # 电机命令变化率上限[1/s]
    config.actuator.servo_rate_limit = 10.0  # 舵机命令变化率上限[1/s]
    config.actuator.vector_origin_body_m = (0.0, 0.0, -0.3)  # 矢量单元在机体系中的安装位置[m]
    config.actuator.motor_thrust_gain = (6.2, 6.2)  # 正/反桨电机推力增益(N/归一化命令)
    config.actuator.motor_thrust_exp = 1.0  # 电机推力非线性指数
    config.actuator.motor_axis_torque_gain = (0.06, 0.06)  # 电机轴向反扭矩增益(N*m/归一化命令)
