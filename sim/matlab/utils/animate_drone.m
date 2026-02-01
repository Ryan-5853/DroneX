function animate_drone(pos, q_hist, t, p)
%% animate_drone - 3D 无人机姿态动画
% pos: 3xN 位置
% q_hist: 4xN 四元数
% t: 1xN 时间
% p: 参数（可选，用于尺寸）

if nargin < 4, p = struct(); end

N = size(pos, 2);
% 降采样以加快动画
skip = max(1, floor(N / 100));
idx = 1:skip:N;
if idx(end) ~= N, idx = [idx, N]; end

figure('Name', 'DroneX 3D 动画', 'Position', [150 150 800 600]);

% 机体外形：简化为一个矩形框 + 推力方向箭头
% 机体尺寸（米）
L = 0.3;  % 半长
W = 0.15; % 半宽
H = 0.1;  % 半高
% 机体角点（机体系，以重心为原点，Z 向下）
corners_body = [L L -L -L L L -L -L;
                W -W -W W W -W -W W;
                H H H H -H -H -H -H] / 2;

h_traj = plot3(pos(1,:), pos(2,:), pos(3,:), 'b-', 'LineWidth', 0.5);
hold on;

% 机体框
h_body = [];
h_thrust = [];

xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('DroneX 姿态动画');
grid on; axis equal;

% 设置坐标轴范围
margin = 1;
xlim([min(pos(1,:))-margin, max(pos(1,:))+margin]);
ylim([min(pos(2,:))-margin, max(pos(2,:))+margin]);
zlim([min(pos(3,:))-margin, max(pos(3,:))+margin]);
view(45, 25);

for i = 1:length(idx)
    k = idx(i);
    R = quat2rotm(q_hist(:,k));
    corners_I = pos(:,k) + R * corners_body;

    % 删除旧的机体
    delete(h_body);
    delete(h_thrust);

    % 绘制机体：6 个面
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    h_body = [];
    for f = 1:6
        fc = faces(f,:);
        h_body(end+1) = patch(corners_I(1,fc), corners_I(2,fc), corners_I(3,fc), ...
            [0.3 0.6 0.9], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
    end

    % 推力方向箭头（推力方向，惯性系向上）
    thrust_dir = R * [0; 0; 1];  % 机体系 +Z，惯性系
    arrow_len = 0.2;
    arrow_end = pos(:,k) + arrow_len * thrust_dir;
    h_thrust = plot3([pos(1,k), arrow_end(1)], [pos(2,k), arrow_end(2)], [pos(3,k), arrow_end(3)], ...
        'r-', 'LineWidth', 3);

    title(sprintf('t = %.2f s', t(k)));
    drawnow;
    pause(0.02);
end

hold off;
fprintf('动画结束。\n');
