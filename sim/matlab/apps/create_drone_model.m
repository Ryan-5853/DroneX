function h = create_drone_model(ax)
%% create_drone_model - 在指定 axes 中创建 DroneX 3D 模型
% 使用 hgtransform 层次结构，支持高效实时更新
%
% 输入:
%   ax: 目标 axes 句柄
%
% 输出:
%   h: 结构体，包含所有图形对象句柄
%       h.root_tf      - 根变换 (位置平移)
%       h.body_tf      - 机体变换 (姿态旋转)
%       h.gimbal_tf    - 云台变换 (alpha,beta 旋转)
%       h.trail        - 轨迹 animatedline
%       h.target       - 目标标记 plot3
%       h.thrust_arrow - 推力箭头句柄
%
% 使用方式:
%   h = create_drone_model(ax);
%   set(h.root_tf, 'Matrix', makehgtform('translate', [x y z]));
%   T_body = eye(4); T_body(1:3,1:3) = R_IB;
%   set(h.body_tf, 'Matrix', T_body);

hold(ax, 'on');

%% 地面网格
ground_size = 12;
ground_div = 24;
gv = linspace(-ground_size/2, ground_size/2, ground_div+1);
[gx, gy] = meshgrid(gv, gv);
gz = zeros(size(gx));
mesh(ax, gx, gy, gz, 'FaceAlpha', 0.08, 'EdgeColor', [0.6 0.6 0.6], ...
    'EdgeAlpha', 0.3, 'FaceColor', [0.9 0.9 0.9]);

%% hgtransform 层次
%   ax
%   └── root_tf (平移: 惯性系位置)
%       └── body_tf (旋转: R_IB)
%           ├── 机体 patch
%           ├── 坐标轴 line
%           ├── 前方标记 patch
%           └── gimbal_base_tf (平移: [0,0,-l_T])
%               └── gimbal_tf (旋转: 云台角)
%                   ├── 云台圆柱 patch
%                   ├── 推力箭头 line
%                   └── 桨盘 patch

h.root_tf = hgtransform('Parent', ax);
h.body_tf = hgtransform('Parent', h.root_tf);

%% === 机体 (Parent: body_tf) ===
% 细长型机体: 10x10x50 cm (X x Y x Z), 重心在几何中心
bL = 0.10; bW = 0.10; bH = 0.50;
[bv, bf] = box_geometry(bL, bW, bH, [0; 0; 0]);
patch('Vertices', bv, 'Faces', bf, ...
    'FaceColor', [0.2 0.5 0.85], 'FaceAlpha', 0.7, ...
    'EdgeColor', [0.1 0.3 0.6], 'LineWidth', 0.5, ...
    'Parent', h.body_tf);

%% 机体坐标轴 (Parent: body_tf)
axis_len = 0.18;
line([0 axis_len], [0 0], [0 0], 'Color','r', 'LineWidth',2.5, 'Parent',h.body_tf);
line([0 0], [0 axis_len], [0 0], 'Color',[0 0.7 0], 'LineWidth',2.5, 'Parent',h.body_tf);
line([0 0], [0 0], [0 axis_len], 'Color','b', 'LineWidth',2.5, 'Parent',h.body_tf);

%% 前方指示 (小三角 patch, 位于机体前面中部)
nose_x = [bL/2; bL/2+0.04; bL/2];
nose_y = [0.02; 0; -0.02];
nose_z = [0; 0; 0];
patch('Vertices', [nose_x, nose_y, nose_z], 'Faces', [1 2 3], ...
    'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.9, 'EdgeColor', 'none', ...
    'Parent', h.body_tf);

%% === 云台 (Parent: body_tf -> gimbal_base_tf -> gimbal_tf) ===
% 云台底座平移到机体底部 (z = -bH/2 = -0.25)
h.gimbal_base_tf = hgtransform('Parent', h.body_tf);
set(h.gimbal_base_tf, 'Matrix', makehgtform('translate', [0, 0, -0.25]));

% 云台旋转节点
h.gimbal_tf = hgtransform('Parent', h.gimbal_base_tf);

% 云台圆柱 (略大一点，匹配细长机体)
n_cyl = 16;
[cyl_v, cyl_f] = cylinder_geometry(0.035, 0.07, n_cyl);
patch('Vertices', cyl_v, 'Faces', cyl_f, ...
    'FaceColor', [0.55 0.55 0.55], 'FaceAlpha', 0.85, ...
    'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 0.3, ...
    'Parent', h.gimbal_tf);

% 推力箭头 (沿 +Z, 用 line 代替 quiver3)
h.thrust_arrow = line([0 0], [0 0], [0 0.3], ...
    'Color', [0.9 0.15 0.15], 'LineWidth', 3.5, ...
    'Parent', h.gimbal_tf);
% 箭头头部 (小三角形)
h.thrust_head = patch('Vertices', [0.015 0 0.28; -0.015 0 0.28; 0 0 0.32; ...
                                    0 0.015 0.28; 0 -0.015 0.28; 0 0 0.32], ...
    'Faces', [1 2 3; 4 5 6], ...
    'FaceColor', [0.9 0.15 0.15], 'FaceAlpha', 0.9, 'EdgeColor', 'none', ...
    'Parent', h.gimbal_tf);

% 桨盘指示 (半透明圆)
theta_disk = linspace(0, 2*pi, 24);
disk_r = 0.12;
disk_x = disk_r * cos(theta_disk);
disk_y = disk_r * sin(theta_disk);
disk_z = -0.035 * ones(size(theta_disk));
patch('XData', disk_x, 'YData', disk_y, 'ZData', disk_z, ...
    'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.15, ...
    'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 0.5, ...
    'Parent', h.gimbal_tf);

%% === 轨迹线 (用 line 代替 animatedline，兼容性更好) ===
h.trail = line(ax, NaN, NaN, NaN, 'Color', [0.2 0.6 1.0], 'LineWidth', 1.2);

%% === 目标标记 ===
h.target = plot3(ax, 0, 0, 5, 'p', 'Color', [0.1 0.8 0.2], ...
    'MarkerSize', 14, 'MarkerFaceColor', [0.2 0.9 0.3], 'LineWidth', 1.5);

%% Axes 配置
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-6 6]); ylim(ax, [-6 6]); zlim(ax, [-0.5 10]);
view(ax, 135, 25);
ax.Projection = 'perspective';
rotate3d(ax, 'on');

hold(ax, 'off');
end

%% ========== 辅助函数 ==========

function [v, f] = box_geometry(L, W, H, center)
%BOX_GEOMETRY 长方体顶点和面
cx = center(1); cy = center(2); cz = center(3);
hl = L/2; hw = W/2; hh = H/2;
v = [cx-hl, cy-hw, cz-hh;
     cx+hl, cy-hw, cz-hh;
     cx+hl, cy+hw, cz-hh;
     cx-hl, cy+hw, cz-hh;
     cx-hl, cy-hw, cz+hh;
     cx+hl, cy-hw, cz+hh;
     cx+hl, cy+hw, cz+hh;
     cx-hl, cy+hw, cz+hh];
f = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
end

function [v, f] = cylinder_geometry(radius, height, n)
%CYLINDER_GEOMETRY 圆柱顶点和面 (中心在原点, 沿Z轴)
theta = linspace(0, 2*pi, n+1);
theta(end) = [];
top_z = height/2;
bot_z = -height/2;

% 顶点: 上圈(1..n), 下圈(n+1..2n), 上中心(2n+1), 下中心(2n+2)
vx = [radius*cos(theta)'; radius*cos(theta)'; 0; 0];
vy = [radius*sin(theta)'; radius*sin(theta)'; 0; 0];
vz = [top_z*ones(n,1); bot_z*ones(n,1); top_z; bot_z];
v = [vx, vy, vz];

% 面
f = zeros(3*n, 4);
for i = 1:n
    j = mod(i, n) + 1;
    f(3*(i-1)+1, :) = [i, j, n+j, n+i];           % 侧面
    f(3*(i-1)+2, :) = [2*n+1, j, i, i];            % 顶面
    f(3*(i-1)+3, :) = [2*n+2, n+i, n+j, n+j];      % 底面
end
end
