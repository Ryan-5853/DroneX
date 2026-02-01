function q = quat_normalize(q)
%% quat_normalize - 四元数归一化
q = q(:) / norm(q);
