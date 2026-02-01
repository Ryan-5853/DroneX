function [roll, pitch, yaw] = quat2euler(q)
%% quat2euler - 四元数转欧拉角 (ZYX)
% q = [w; x; y; z]
% 输出单位 rad

q = q(:);
w = q(1); x = q(2); y = q(3); z = q(4);

% 提取欧拉角
sinp = 2*(w*y - z*x);
if abs(sinp) >= 1
    pitch = sign(sinp) * pi/2;
else
    pitch = asin(sinp);
end

sinr_cosp = 2*(w*x + y*z);
cosr_cosp = 1 - 2*(x^2 + y^2);
roll = atan2(sinr_cosp, cosr_cosp);

siny_cosp = 2*(w*z + x*y);
cosy_cosp = 1 - 2*(y^2 + z^2);
yaw = atan2(siny_cosp, cosy_cosp);
