%% init_path - 添加 DroneX 仿真 MATLAB 路径
% 在 MATLAB 中运行: cd sim/matlab; init_path;
% 或: run('sim/matlab/init_path.m')

sim_root = fileparts(mfilename('fullpath'));
addpath(fullfile(sim_root, 'models'));
addpath(fullfile(sim_root, 'controllers'));
addpath(fullfile(sim_root, 'utils'));
addpath(fullfile(sim_root, 'scripts'));
addpath(fullfile(sim_root, 'apps'));
fprintf('DroneX 仿真路径已添加: %s\n', sim_root);
