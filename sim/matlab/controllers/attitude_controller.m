function [alpha, beta, state_out] = attitude_controller(q, omega, att_des, T, state, p)
%% attitude_controller - 姿态控制器，输出云台角
% q: 当前姿态四元数
% omega: 机体系角速度 [rad/s]
% att_des: 期望姿态 [roll; pitch; yaw] rad
% T: 推力幅值 (N)，由外环位置控制器提供
% state: 控制器内部状态，初次传 struct('att_int',zeros(3,1),'rate_int',zeros(3,1))
% p: 参数结构体
% alpha, beta: 云台角输出
% state_out: 更新后的状态

if isempty(state) || ~isfield(state, 'att_int')
    att_int = zeros(3,1);
    rate_int = zeros(3,1);
else
    att_int = state.att_int;
    rate_int = state.rate_int;
end

dt = p.dt;

% 当前欧拉角
[roll, pitch, yaw] = quat2euler(q);
att = [roll; pitch; yaw];

% 角度误差（处理 ±π wrap）
att_err = att_des - att;
att_err = atan2(sin(att_err), cos(att_err));

% 外环：姿态角 -> 期望角速度
omega_des = p.att_Kp .* att_err + p.att_Ki .* att_int;
att_int = att_int + att_err * dt;

% 内环：角速度 -> 期望力矩
rate_err = omega_des - omega;
tau_des = p.rate_Kp .* rate_err + p.rate_Ki .* rate_int;
rate_int = rate_int + rate_err * dt;

% 控制分配：tau_des -> (alpha, beta)，T 由外环提供
% 线性化：tau_x ≈ T*d*beta, tau_y ≈ -T*d*alpha
d = abs(p.r_thrust(3));
T_use = max(T, 1e-6);

if d > 1e-6 && T_use > 1e-6
    alpha = -tau_des(2) / (T_use * d);
    beta  =  tau_des(1) / (T_use * d);
    alpha = max(-p.alpha_lim, min(p.alpha_lim, alpha));
    beta  = max(-p.beta_lim, min(p.beta_lim, beta));
else
    alpha = 0; beta = 0;
end

state_out = struct('att_int', att_int, 'rate_int', rate_int);
