%% params.m - DroneX 仿真参数配置
% 刚体、推进器、云台、控制器参数
% 在 run_basic_sim 等脚本中调用: params;

%% 物理常数
g = 9.81;  % m/s^2

%% 刚体参数 (p 为参数结构体，供模型与控制器使用)
p = struct();
p.g = g;
p.m = 1.5;              % 质量 kg
p.I = diag([0.02, 0.02, 0.03]);  % 转动惯量 kg·m² (Ixx, Iyy, Izz)
p.I_inv = inv(p.I);

%% 几何参数 - 推力作用点相对重心 (机体系)
% 重心在推力点之上，Z 分量为负（推力点在 -Z_B 方向）
p.r_thrust = [0; 0; -0.15];  % m，推力点距重心 15 cm

%% 云台参数
p.alpha_lim = deg2rad(25);   % 俯仰角限位 rad
p.beta_lim = deg2rad(25);   % 滚转角限位 rad
p.tau_gimbal = 0.05;        % 云台一阶响应时间常数 s

%% 推进器参数
% 悬停推力 ≈ m*g
p.T_hover = p.m * g;
p.T_max = 2.5 * p.T_hover;  % 最大推力
p.T_min = 0.1 * p.T_hover;  % 最小推力

%% 姿态控制器 PID 参数（串级：外环姿态角，内环角速度）
% 外环：姿态角误差 -> 期望角速度
p.att_Kp = [4; 4; 2];       % 俯仰、滚转、偏航
p.att_Ki = [0.5; 0.5; 0.2];
p.att_Kd = [0; 0; 0];

% 内环：角速度误差 -> 力矩
p.rate_Kp = [0.08; 0.08; 0.05];
p.rate_Ki = [0.02; 0.02; 0.01];
p.rate_Kd = [0; 0; 0];

%% 仿真参数
p.dt = 0.002;    % 控制周期 s (500 Hz)
p.t_end = 10;    % 仿真时长 s
