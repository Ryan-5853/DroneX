%% run_verify_hover - 悬停与平移验证（verification.md 用例）
% 用例1: 定点悬停 (0,0,5) 30s
% 用例2: 水平平移 (0,0,5) -> (2,0,5)

clear; clc; close all;

%% 添加路径
sim_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(sim_root, 'models'));
addpath(fullfile(sim_root, 'controllers'));
addpath(fullfile(sim_root, 'utils'));

%% 加载参数
run(fullfile(sim_root, 'scripts', 'params.m'));
p.t_end = 30;  % 验证用例 30 s

%% 选择场景: 1=定点悬停, 2=水平平移
scenario = 2;  % 改为 1 或 2

%% 初始状态
q0 = euler2quat(deg2rad(3), deg2rad(3), 0);
q0 = quat_normalize(q0);
x0 = [0; 0; 5; 0; 0; 0; q0; 0; 0; 0];

%% 仿真
dt = p.dt;
t_end = p.t_end;
t = 0:dt:t_end;
N = length(t);

X = zeros(13, N);
U = zeros(3, N);

X(:,1) = x0;
pos_state = struct('pos_int', zeros(3,1));
att_state = struct('att_int', zeros(3,1), 'rate_int', zeros(3,1));

for k = 1:N-1
    x = X(:,k);
    pos = x(1:3);
    vel = x(4:6);
    q = x(7:10);
    omega = x(11:13);

    % 期望轨迹
    if scenario == 1
        pos_des = [0; 0; 5];
        vel_des = [0; 0; 0];
    else
        % 场景2: t<5 悬停，t>=5 目标 (2,0,5)
        if t(k) < 5
            pos_des = [0; 0; 5];
            vel_des = [0; 0; 0];
        else
            pos_des = [2; 0; 5];
            vel_des = [0; 0; 0];
        end
    end

    [T, att_des, pos_state] = position_controller(pos, vel, pos_des, vel_des, pos_state, p);
    [alpha, beta, att_state] = attitude_controller(q, omega, att_des, T, att_state, p);
    u = [T; alpha; beta];
    U(:,k) = u;

    k1 = rigid_body_ode(t(k), x, u, p);
    k2 = rigid_body_ode(t(k)+dt/2, x+dt/2*k1, u, p);
    k3 = rigid_body_ode(t(k)+dt/2, x+dt/2*k2, u, p);
    k4 = rigid_body_ode(t(k)+dt, x+dt*k3, u, p);
    x_next = x + (dt/6)*(k1+2*k2+2*k3+k4);
    x_next(7:10) = quat_normalize(x_next(7:10));
    X(:,k+1) = x_next;
end
U(:,N) = U(:,N-1);

%% 提取结果
pos = X(1:3,:);
vel = X(4:6,:);
q_hist = X(7:10,:);
roll_hist = zeros(1,N);
pitch_hist = zeros(1,N);
yaw_hist = zeros(1,N);
for k = 1:N
    [roll_hist(k), pitch_hist(k), yaw_hist(k)] = quat2euler(q_hist(:,k));
end

%% 绘图
scenario_name = {'定点悬停', '水平平移 (0,0,5)->(2,0,5)'};
figure('Name', ['悬停验证: ' scenario_name{scenario}], 'Position', [50 50 1000 700]);

subplot(2,2,1);
plot(t, pos(1,:), t, pos(2,:), t, pos(3,:));
legend('x','y','z'); xlabel('t (s)'); ylabel('位置 (m)'); title('位置');
grid on;

subplot(2,2,2);
plot(t, rad2deg(roll_hist), t, rad2deg(pitch_hist));
legend('roll','pitch'); xlabel('t (s)'); ylabel('姿态角 (°)'); title('姿态');
grid on;

subplot(2,2,3);
plot(t, U(1,:)); xlabel('t (s)'); ylabel('T (N)'); title('推力');
grid on;

subplot(2,2,4);
plot3(pos(1,:), pos(2,:), pos(3,:), 'b-');
hold on;
plot3(pos(1,1), pos(2,1), pos(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(pos(1,end), pos(2,end), pos(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('x'); ylabel('y'); zlabel('z'); title('3D 轨迹');
grid on; axis equal; view(45, 25);

%% 稳态误差（最后 5s 平均）
idx_ss = t >= (t_end - 5);
if scenario == 1
    pos_des_final = [0; 0; 5];
else
    pos_des_final = [2; 0; 5];
end
pos_err = pos_des_final - [mean(pos(1,idx_ss)); mean(pos(2,idx_ss)); mean(pos(3,idx_ss))];
vel_ss = [mean(abs(vel(1,idx_ss))); mean(abs(vel(2,idx_ss))); mean(abs(vel(3,idx_ss)))];
fprintf('\n=== 验证: %s ===\n', scenario_name{scenario});
fprintf('稳态位置误差 (最后5s): x=%.3f y=%.3f z=%.3f m\n', pos_err(1), pos_err(2), pos_err(3));
fprintf('稳态速度幅值: vx=%.3f vy=%.3f vz=%.3f m/s\n', vel_ss(1), vel_ss(2), vel_ss(3));
fprintf('稳态姿态: roll=%.1f pitch=%.1f deg\n', rad2deg(mean(abs(roll_hist(idx_ss)))), rad2deg(mean(abs(pitch_hist(idx_ss)))));
