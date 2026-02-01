function [T, att_des, state_out] = position_controller(pos, vel, pos_des, vel_des, state, p)
%% position_controller - 位置控制器，输出推力幅值与期望姿态
% pos, vel: 当前位置、速度（惯性系）
% pos_des, vel_des: 期望位置、速度
% state: 内部状态，初次传 struct('pos_int',zeros(3,1))
% p: 参数结构体
% T: 推力幅值 (N)
% att_des: 期望姿态 [roll; pitch; yaw] rad
% state_out: 更新后的状态

if isempty(state) || ~isfield(state, 'pos_int')
    pos_int = zeros(3,1);
else
    pos_int = state.pos_int;
end

dt = p.dt;
g_vec = [0; 0; -p.g];

% 位置与速度误差
pos_err = pos_des(:) - pos(:);
vel_err = vel_des(:) - vel(:);

% 期望加速度（PID）
a_des = p.pos_Kp .* pos_err + p.pos_Ki .* pos_int + p.pos_Kd .* vel_err;

% 积分抗饱和：仅当未饱和时累计
pos_int = pos_int + pos_err * dt;
% 限幅防止积分饱和
pos_int = max(-p.pos_int_lim, min(p.pos_int_lim, pos_int));

% 期望推力（惯性系）：m*a = F + m*g => F = m*(a_des - g)
F_des = p.m * (a_des - g_vec);

% 推力幅值与限幅
T = norm(F_des);
T = max(p.T_min, min(p.T_max, T));

% 推力方向（惯性系）
if T > 1e-6
    d = F_des / T;
else
    d = [0; 0; 1];  % 默认向上
end

% 方向 -> 期望姿态
att_des = thrust_dir_to_att(d);

state_out = struct('pos_int', pos_int);
