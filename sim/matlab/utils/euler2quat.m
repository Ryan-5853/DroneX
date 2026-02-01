function q = euler2quat(roll, pitch, yaw)
%% euler2quat - 欧拉角 (ZYX 内旋) 转四元数
% roll, pitch, yaw 单位 rad
% q = [w; x; y; z]

cr = cos(roll/2);  sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2);   sy = sin(yaw/2);

w = cr*cp*cy + sr*sp*sy;
x = sr*cp*cy - cr*sp*sy;
y = cr*sp*cy + sr*cp*sy;
z = cr*cp*sy - sr*sp*cy;

q = [w; x; y; z];
