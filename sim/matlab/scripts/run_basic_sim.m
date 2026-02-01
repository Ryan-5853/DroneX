%% run_basic_sim - DroneX 基础仿真与可视化
% 添加路径 -> 加载参数 -> 运行仿真 -> 绘图与 3D 动画

clear; clc; close all;

%% 添加路径
sim_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(sim_root, 'models'));
addpath(fullfile(sim_root, 'controllers'));
addpath(fullfile(sim_root, 'utils'));

%% 加载参数
run(fullfile(sim_root, 'scripts', 'params.m'));

%% 初始状态
% 状态: [px; py; pz; vx; vy; vz; qw; qx; qy; qz; omx; omy; omz]
% 初始位置 (0,0,5m)，小幅俯仰扰动以观察稳定
roll0 = deg2rad(5);
pitch0 = deg2rad(5);
yaw0 = 0;
q0 = euler2quat(roll0, pitch0, yaw0);
q0 = quat_normalize(q0);

x0 = [0; 0; 5; 0; 0; 0; q0; 0; 0; 0];

%% 仿真
dt = p.dt;
t_end = p.t_end;
t = 0:dt:t_end;
N = length(t);

X = zeros(13, N);
U = zeros(3, N);
att_des = [0; 0; 0];  % 期望水平

X(:,1) = x0;
ctrl_state = struct('att_int', zeros(3,1), 'rate_int', zeros(3,1));

for k = 1:N-1
    x = X(:,k);
    q = x(7:10);
    omega = x(11:13);

    [T, alpha, beta, ctrl_state] = attitude_controller(q, omega, att_des, ctrl_state, p);
    u = [T; alpha; beta];
    U(:,k) = u;

    % 固定步长 RK4 积分
    k1 = rigid_body_ode(t(k), x, u, p);
    k2 = rigid_body_ode(t(k)+dt/2, x+dt/2*k1, u, p);
    k3 = rigid_body_ode(t(k)+dt/2, x+dt/2*k2, u, p);
    k4 = rigid_body_ode(t(k)+dt, x+dt*k3, u, p);
    x_next = x + (dt/6)*(k1+2*k2+2*k3+k4);

    % 四元数归一化
    x_next(7:10) = quat_normalize(x_next(7:10));
    X(:,k+1) = x_next;
end
U(:,N) = U(:,N-1);

%% 提取轨迹与姿态
pos = X(1:3,:);
vel = X(4:6,:);
q_hist = X(7:10,:);
omega_hist = X(11:13,:);

roll_hist = zeros(1,N);
pitch_hist = zeros(1,N);
yaw_hist = zeros(1,N);
for k = 1:N
    [roll_hist(k), pitch_hist(k), yaw_hist(k)] = quat2euler(q_hist(:,k));
end

%% 绘图 1: 时域曲线
figure('Name', 'DroneX 仿真 - 时域', 'Position', [50 50 1200 800]);

subplot(3,2,1);
plot(t, pos(1,:), t, pos(2,:), t, pos(3,:));
legend('x','y','z');
xlabel('t (s)'); ylabel('位置 (m)');
title('位置');
grid on;

subplot(3,2,2);
plot(t, rad2deg(roll_hist), t, rad2deg(pitch_hist), t, rad2deg(yaw_hist));
legend('roll','pitch','yaw');
xlabel('t (s)'); ylabel('姿态角 (°)');
title('姿态角');
grid on;

subplot(3,2,3);
plot(t, U(1,:));
xlabel('t (s)'); ylabel('T (N)');
title('推力');
grid on;

subplot(3,2,4);
plot(t, rad2deg(U(2,:)), t, rad2deg(U(3,:)));
legend('\alpha','\beta');
xlabel('t (s)'); ylabel('云台角 (°)');
title('云台角');
grid on;

subplot(3,2,5);
plot(t, vel(1,:), t, vel(2,:), t, vel(3,:));
legend('vx','vy','vz');
xlabel('t (s)'); ylabel('速度 (m/s)');
title('速度');
grid on;

subplot(3,2,6);
plot(t, rad2deg(omega_hist(1,:)), t, rad2deg(omega_hist(2,:)), t, rad2deg(omega_hist(3,:)));
legend('\omega_x','\omega_y','\omega_z');
xlabel('t (s)'); ylabel('角速度 (°/s)');
title('角速度');
grid on;

%% 绘图 2: 3D 轨迹
figure('Name', 'DroneX 仿真 - 3D 轨迹', 'Position', [100 100 700 600]);
plot3(pos(1,:), pos(2,:), pos(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot3(pos(1,1), pos(2,1), pos(3,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pos(1,end), pos(2,end), pos(3,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D 轨迹 (绿:起点 红:终点)');
grid on; axis equal;
view(45, 30);

%% 绘图 3: 3D 动画
animate_drone(pos, q_hist, t, p);

%% 保存结果（可选）
% save(fullfile(sim_root, '..', 'data', 'results', 'sim_basic_result.mat'), 't', 'X', 'U', 'p');
fprintf('仿真完成。时长 %.1f s，步数 %d。\n', t_end, N);
