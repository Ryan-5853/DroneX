function [F_thrust_I, tau_thrust] = thrust_model(T, alpha, beta, q, p)
%% thrust_model - 计算推力矢量与力矩
% T: 推力幅值 (N)，标量
% alpha: 云台俯仰角 (rad)
% beta: 云台滚转角 (rad)
% q: 机体姿态四元数 [w;x;y;z]
% p: 参数结构体
% F_thrust_I: 惯性系下推力矢量
% tau_thrust: 机体系下推力产生的力矩

% 云台限位
alpha = max(-p.alpha_lim, min(p.alpha_lim, alpha));
beta = max(-p.beta_lim, min(p.beta_lim, beta));

% 推力方向（机体系）：无偏转时为 [0;0;+1]（沿 Z_B，惯性系向上）
% 约定：零欧拉角时 body=inertial，Z 向上，推力沿 +Z
% 俯仰 alpha 绕 Y，滚转 beta 绕 X：d_body = R_x(-beta)*R_y(alpha)*[0;0;1]
% 正 alpha -> 推力偏向 +X_B；正 beta -> 推力偏向 +Y_B
% 参考: sim/docs/dynamics_model.md §3.5.1
sa = sin(alpha); ca = cos(alpha);
sb = sin(beta);  cb = cos(beta);
d_body = [sa; sb*ca; cb*ca];  % 单位向量

F_thrust_body = T * d_body;

% 转换到惯性系
R_IB = quat2rotm(q);
F_thrust_I = R_IB * F_thrust_body;

% 力矩：τ = r × F，r 为推力点相对重心的矢量（机体系）
r = p.r_thrust;
tau_thrust = cross(r, F_thrust_body);
