function R = quat2rotm(q)
%% quat2rotm - 四元数转旋转矩阵（机体系到惯性系）
% q = [w; x; y; z]，标量在前的 Hamilton 约定
% R: body to inertial, v_I = R * v_B

q = q(:);
w = q(1); x = q(2); y = q(3); z = q(4);

R = [1-2*(y^2+z^2),  2*(x*y-w*z),    2*(x*z+w*y);
     2*(x*y+w*z),    1-2*(x^2+z^2),  2*(y*z-w*x);
     2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x^2+y^2)];
