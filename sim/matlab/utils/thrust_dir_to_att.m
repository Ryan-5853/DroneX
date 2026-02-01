function att_des = thrust_dir_to_att(d)
%% thrust_dir_to_att - 推力方向（惯性系单位向量）转期望姿态角
% d: 惯性系下推力方向 [3x1]，单位向量，应近似向上 (d(3)>0)
% att_des: [roll; pitch; yaw] rad，yaw=0（偏航由共轴桨差速等实现）
% body Z = Ry(pitch)*Rx(roll)*e3 = [sin(pitch)*cos(roll); -sin(roll); cos(pitch)*cos(roll)] = d
% 反解：pitch = atan2(dx, dz), roll = atan2(-dy, dz)

d = d(:);
d = d / (norm(d) + 1e-10);

% 避免奇异
dz = max(0.2, min(1, d(3)));  % 至少约 12° 仰角
dx = d(1); dy = d(2);

pitch_des = atan2(dx, dz);   % 推力 +x 需 pitch>0（机头下压）
roll_des  = atan2(-dy, dz);  % 推力 +y 需 roll<0
yaw_des   = 0;

att_des = [roll_des; pitch_des; yaw_des];
