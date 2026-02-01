function xdot = rigid_body_ode(t, x, u, p)
%% rigid_body_ode - 六自由度刚体动力学 ODE
% 状态 x: [px; py; pz; vx; vy; vz; qw; qx; qy; qz; omx; omy; omz]
% 控制 u: [T; alpha; beta]
% p: 参数结构体
% xdot: 状态导数

px = x(1); py = x(2); pz = x(3);
vx = x(4); vy = x(5); vz = x(6);
q = x(7:10);
omega = x(11:13);

T = u(1); alpha = u(2); beta = u(3);

% 推力与力矩
[F_thrust_I, tau_thrust] = thrust_model(T, alpha, beta, q, p);

% 重力
g_vec = [0; 0; -p.g];
F_grav = p.m * g_vec;

% 平动加速度 (惯性系)
a_I = (F_thrust_I + F_grav) / p.m;
p_dot = [vx; vy; vz];
v_dot = a_I;

% 四元数导数: q_dot = 0.5 * Omega(omega) * q
% Omega(omega) = [0 -omx -omy -omz; omx 0 omz -omy; omy -omz 0 omx; omz omy -omx 0]
Om = [0,    -omega(1), -omega(2), -omega(3);
      omega(1), 0,       omega(3), -omega(2);
      omega(2), -omega(3), 0,       omega(1);
      omega(3),  omega(2), -omega(1), 0];
q_dot = 0.5 * Om * q;

% 角加速度 (欧拉方程): I*omega_dot + omega × (I*omega) = tau
I_omega = p.I * omega;
omega_dot = p.I_inv * (tau_thrust - cross(omega, I_omega));

xdot = [p_dot; v_dot; q_dot; omega_dot];
