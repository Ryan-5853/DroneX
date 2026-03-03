classdef DroneXSimApp < handle
%% DroneXSimApp - DroneX 交互式 3D 仿真应用
% 用法:
%   cd sim/matlab; init_path; app = DroneXSimApp();
%
% 功能:
%   - 3D 可视化: 机体、云台、推力矢量、轨迹线、目标标记
%   - 自由摄像机: 旋转/缩放/平移 (rotate3d)
%   - 交互控件: 目标位置、扰动力、PID 参数实时调节
%   - 实时曲线: 位置、姿态角、控制量滚动绘图
%   - 仿真控制: 启动/暂停/重置、速度调节

    %% ===== 属性 =====
    properties
        % 图形窗口
        fig

        % 3D 视图
        ax3d
        model

        % 时域曲线 axes
        axPos
        axAtt
        axCtrl

        % 时域曲线 line 句柄 (各 3 条)
        linePos
        lineAtt
        lineCtrl

        % 时域数据缓冲区
        buf_t       % 时间戳 [Nx1]
        buf_pos     % 位置 [Nx3]
        buf_att     % 姿态角 deg [Nx3]
        buf_ctrl    % 控制量 [Nx3]: T(N), alpha(deg), beta(deg)
        buf_idx     % 当前写入位置
        buf_max     % 缓冲区最大长度

        % 3D 轨迹数据
        trail_buf   % [Nx3]
        trail_idx

        % UI 控件句柄
        btnStart
        btnStop
        btnReset
        sliderSpeed
        lblTime
        lblInfo

        sliderTargetX
        sliderTargetY
        sliderTargetZ
        lblTargetX
        lblTargetY
        lblTargetZ

        sliderDistFx
        sliderDistFy
        lblDistFx
        lblDistFy

        sliderAttKp
        sliderRateKp
        lblAttKp
        lblRateKp

        sliderPosKp
        sliderPosKd
        lblPosKp
        lblPosKd

        % 仿真状态
        p
        x
        t_sim
        pos_state
        att_state
        last_u
        running

        % Timer
        sim_timer

        % 显示时间窗口
        plot_window
    end

    %% ===== 构造 / 析构 =====
    methods
        function app = DroneXSimApp()
            app.running = false;
            app.plot_window = 15;
            app.last_u = [0; 0; 0];

            % 数据缓冲区
            app.buf_max = 5000;
            app.buf_t    = NaN(app.buf_max, 1);
            app.buf_pos  = NaN(app.buf_max, 3);
            app.buf_att  = NaN(app.buf_max, 3);
            app.buf_ctrl = NaN(app.buf_max, 3);
            app.buf_idx  = 0;

            app.trail_buf = NaN(3000, 3);
            app.trail_idx = 0;

            % 加载参数
            app.loadParams();
            app.resetState();

            % UI + 3D 模型 + 曲线
            app.createUI();
            app.model = create_drone_model(app.ax3d);
            app.createPlotLines();
            app.updateVisuals();
            drawnow;

            % Timer (50 Hz)
            app.sim_timer = timer('ExecutionMode', 'fixedRate', ...
                'Period', 0.02, ...
                'BusyMode', 'drop', ...
                'TimerFcn', @(~,~) app.timerCallback(), ...
                'ErrorFcn', @(~,e) app.onTimerError(e));
        end

        function delete(app)
            if ~isempty(app.sim_timer)
                try stop(app.sim_timer); catch, end
                try delete(app.sim_timer); catch, end
            end
        end
    end

    %% ===== 回调方法 (public) =====
    methods
        function timerCallback(app)
            if ~app.running, return; end
            if ~isvalid(app.fig), app.onStop(); return; end

            try
                app.readUIParams();

                sim_speed = get(app.sliderSpeed, 'Value');
                dt = app.p.dt;
                n_steps = max(1, round(0.02 * sim_speed / dt));

                pos_des = [get(app.sliderTargetX, 'Value'); ...
                           get(app.sliderTargetY, 'Value'); ...
                           get(app.sliderTargetZ, 'Value')];
                vel_des = [0; 0; 0];

                F_dist = [get(app.sliderDistFx, 'Value'); ...
                          get(app.sliderDistFy, 'Value'); ...
                          0];

                for kk = 1:n_steps
                    pos = app.x(1:3);
                    vel = app.x(4:6);
                    q   = app.x(7:10);
                    omega = app.x(11:13);

                    [T_thrust, att_des, app.pos_state] = position_controller( ...
                        pos, vel, pos_des, vel_des, app.pos_state, app.p);
                    [alpha, beta, app.att_state] = attitude_controller( ...
                        q, omega, att_des, T_thrust, app.att_state, app.p);
                    u = [T_thrust; alpha; beta];
                    app.last_u = u;

                    t_now = app.t_sim;
                    k1 = rigid_body_ode(t_now,        app.x,            u, app.p);
                    k2 = rigid_body_ode(t_now+dt/2,   app.x+dt/2*k1,   u, app.p);
                    k3 = rigid_body_ode(t_now+dt/2,   app.x+dt/2*k2,   u, app.p);
                    k4 = rigid_body_ode(t_now+dt,     app.x+dt*k3,     u, app.p);
                    app.x = app.x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

                    app.x(4:6) = app.x(4:6) + (F_dist / app.p.m) * dt;
                    app.x(7:10) = quat_normalize(app.x(7:10));
                    app.t_sim = app.t_sim + dt;
                end

                % 记录数据点
                app.recordData();

                % 更新所有显示
                app.updateVisuals();
                app.updatePlots();
                app.updateInfoPanel(pos_des);
                app.updateSliderLabels();

            catch ex
                err_msg = sprintf('ERROR: %s\n%s line %d', ...
                    ex.message, ex.stack(1).name, ex.stack(1).line);
                set(app.lblInfo, 'String', err_msg);
                warning('DroneXSimApp:simError', '%s', ex.getReport());
            end

            drawnow limitrate;
        end

        function onTimerError(app, e)
            err_msg = sprintf('TIMER ERROR: %s', e.Data.message);
            if isvalid(app.fig)
                set(app.lblInfo, 'String', err_msg);
                drawnow;
            end
            warning('DroneXSimApp:timerError', '%s', err_msg);
        end

        function onStart(app)
            if ~app.running
                app.running = true;
                set(app.lblInfo, 'String', 'Starting simulation...');
                drawnow;
                try
                    start(app.sim_timer);
                catch ex
                    app.running = false;
                    set(app.lblInfo, 'String', sprintf('Start failed: %s', ex.message));
                    drawnow;
                end
            end
        end

        function onStop(app)
            app.running = false;
            if ~isempty(app.sim_timer)
                try stop(app.sim_timer); catch, end
            end
        end

        function onReset(app)
            app.onStop();
            app.resetState();

            % 清空数据缓冲
            app.buf_t(:)    = NaN;
            app.buf_pos(:)  = NaN;
            app.buf_att(:)  = NaN;
            app.buf_ctrl(:) = NaN;
            app.buf_idx     = 0;
            app.trail_buf(:) = NaN;
            app.trail_idx    = 0;

            % 清空曲线和轨迹
            app.updatePlots();
            set(app.model.trail, 'XData', NaN, 'YData', NaN, 'ZData', NaN);

            % 重置滑块
            set(app.sliderTargetX, 'Value', 0);
            set(app.sliderTargetY, 'Value', 0);
            set(app.sliderTargetZ, 'Value', 5);
            set(app.sliderDistFx, 'Value', 0);
            set(app.sliderDistFy, 'Value', 0);

            app.loadParams();
            set(app.sliderAttKp, 'Value', app.p.att_Kp(1));
            set(app.sliderRateKp, 'Value', app.p.rate_Kp(1));
            set(app.sliderPosKp, 'Value', app.p.pos_Kp(1));
            set(app.sliderPosKd, 'Value', app.p.pos_Kd(1));

            app.updateVisuals();
            app.updateSliderLabels();
            set(app.lblTime, 'String', 't = 0.0 s');
            set(app.lblInfo, 'String', 'Ready. Press Start.');
            drawnow;
        end

        function onClose(app)
            app.onStop();
            if isvalid(app.fig)
                delete(app.fig);
            end
        end
    end

    %% ===== 初始化 (private) =====
    methods (Access = private)
        function loadParams(app)
            run('params.m');
            app.p = p; %#ok<PROP>
        end

        function resetState(app)
            q0 = euler2quat(0, 0, 0);
            q0 = quat_normalize(q0);
            app.x = [0; 0; 5; 0; 0; 0; q0; 0; 0; 0];
            app.t_sim = 0;
            app.pos_state = struct('pos_int', zeros(3,1));
            app.att_state = struct('att_int', zeros(3,1), 'rate_int', zeros(3,1));
            app.last_u = [app.p.m * app.p.g; 0; 0];
        end

        function createUI(app)
            app.fig = figure('Name', 'DroneX Interactive Simulation', ...
                'NumberTitle', 'off', ...
                'Position', [80 60 1400 820], ...
                'MenuBar', 'none', 'ToolBar', 'none', ...
                'Color', [0.94 0.94 0.94], ...
                'CloseRequestFcn', @(~,~) app.onClose(), ...
                'Resize', 'on');

            app.ax3d = axes('Parent', app.fig, ...
                'Units', 'normalized', 'Position', [0.02 0.38 0.62 0.60]);

            app.axPos = axes('Parent', app.fig, ...
                'Units', 'normalized', 'Position', [0.02 0.04 0.20 0.30]);
            title(app.axPos, 'Position (m)'); xlabel(app.axPos, 't(s)');
            grid(app.axPos, 'on'); hold(app.axPos, 'on');

            app.axAtt = axes('Parent', app.fig, ...
                'Units', 'normalized', 'Position', [0.24 0.04 0.20 0.30]);
            title(app.axAtt, 'Attitude (deg)'); xlabel(app.axAtt, 't(s)');
            grid(app.axAtt, 'on'); hold(app.axAtt, 'on');

            app.axCtrl = axes('Parent', app.fig, ...
                'Units', 'normalized', 'Position', [0.46 0.04 0.20 0.30]);
            title(app.axCtrl, 'Control'); xlabel(app.axCtrl, 't(s)');
            grid(app.axCtrl, 'on'); hold(app.axCtrl, 'on');

            panelX = 0.68; panelW = 0.31;

            yy = 0.93;
            uicontrol(app.fig, 'Style','text', 'String','=== Simulation ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);

            yy = yy - 0.04;
            app.btnStart = uicontrol(app.fig, 'Style','pushbutton', 'String','Start', ...
                'Units','normalized', 'Position',[panelX yy 0.08 0.035], ...
                'FontSize',9, 'Callback', @(~,~) app.onStart());
            app.btnStop = uicontrol(app.fig, 'Style','pushbutton', 'String','Stop', ...
                'Units','normalized', 'Position',[panelX+0.09 yy 0.08 0.035], ...
                'FontSize',9, 'Callback', @(~,~) app.onStop());
            app.btnReset = uicontrol(app.fig, 'Style','pushbutton', 'String','Reset', ...
                'Units','normalized', 'Position',[panelX+0.18 yy 0.08 0.035], ...
                'FontSize',9, 'Callback', @(~,~) app.onReset());

            yy = yy - 0.04;
            uicontrol(app.fig, 'Style','text', 'String','Speed:', ...
                'Units','normalized', 'Position',[panelX yy 0.05 0.025], ...
                'FontSize',9, 'HorizontalAlignment','left', 'BackgroundColor',[0.94 0.94 0.94]);
            app.sliderSpeed = uicontrol(app.fig, 'Style','slider', ...
                'Units','normalized', 'Position',[panelX+0.05 yy 0.18 0.025], ...
                'Min',0.1, 'Max',5, 'Value',1.0, 'SliderStep',[0.05 0.2]);
            app.lblTime = uicontrol(app.fig, 'Style','text', 'String','t = 0.00 s', ...
                'Units','normalized', 'Position',[panelX+0.24 yy 0.07 0.025], ...
                'FontSize',9, 'BackgroundColor',[0.94 0.94 0.94]);

            yy = yy - 0.05;
            uicontrol(app.fig, 'Style','text', 'String','=== Target Position ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);
            [app.sliderTargetX, app.lblTargetX, yy] = app.addSliderRow(yy, 'X:', -5, 5, 0, 'm');
            [app.sliderTargetY, app.lblTargetY, yy] = app.addSliderRow(yy, 'Y:', -5, 5, 0, 'm');
            [app.sliderTargetZ, app.lblTargetZ, yy] = app.addSliderRow(yy, 'Z:', 0, 10, 5, 'm');

            yy = yy - 0.02;
            uicontrol(app.fig, 'Style','text', 'String','=== Disturbance ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);
            [app.sliderDistFx, app.lblDistFx, yy] = app.addSliderRow(yy, 'Fx:', -5, 5, 0, 'N');
            [app.sliderDistFy, app.lblDistFy, yy] = app.addSliderRow(yy, 'Fy:', -5, 5, 0, 'N');

            yy = yy - 0.02;
            uicontrol(app.fig, 'Style','text', 'String','=== Attitude PID ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);
            [app.sliderAttKp, app.lblAttKp, yy] = app.addSliderRow(yy, 'att Kp:', 0, 12, app.p.att_Kp(1), '');
            [app.sliderRateKp, app.lblRateKp, yy] = app.addSliderRow(yy, 'rate Kp:', 0, 0.5, app.p.rate_Kp(1), '');

            yy = yy - 0.02;
            uicontrol(app.fig, 'Style','text', 'String','=== Position PID ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);
            [app.sliderPosKp, app.lblPosKp, yy] = app.addSliderRow(yy, 'pos Kp:', 0, 5, app.p.pos_Kp(1), '');
            [app.sliderPosKd, app.lblPosKd, yy] = app.addSliderRow(yy, 'pos Kd:', 0, 5, app.p.pos_Kd(1), '');

            yy = yy - 0.02;
            uicontrol(app.fig, 'Style','text', 'String','=== Info ===', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.03], ...
                'FontWeight','bold', 'FontSize',10, 'BackgroundColor',[0.94 0.94 0.94]);
            yy = yy - 0.12;
            app.lblInfo = uicontrol(app.fig, 'Style','text', ...
                'String', 'Ready. Press Start.', ...
                'Units','normalized', 'Position',[panelX yy panelW 0.12], ...
                'FontSize',9, 'HorizontalAlignment','left', ...
                'FontName','Consolas', 'BackgroundColor',[0.94 0.94 0.94], 'Max', 2);
        end

        function [slider, lbl, yy_out] = addSliderRow(app, yy, name, vmin, vmax, val, unit)
            panelX = 0.68;
            yy = yy - 0.035;
            uicontrol(app.fig, 'Style','text', 'String', name, ...
                'Units','normalized', 'Position',[panelX yy 0.06 0.025], ...
                'FontSize',9, 'HorizontalAlignment','left', 'BackgroundColor',[0.94 0.94 0.94]);
            slider = uicontrol(app.fig, 'Style','slider', ...
                'Units','normalized', 'Position',[panelX+0.06 yy 0.17 0.025], ...
                'Min', vmin, 'Max', vmax, 'Value', val, ...
                'SliderStep', [0.01 0.1]);
            lbl = uicontrol(app.fig, 'Style','text', ...
                'String', sprintf('%.2f %s', val, unit), ...
                'Units','normalized', 'Position',[panelX+0.24 yy 0.07 0.025], ...
                'FontSize',9, 'BackgroundColor',[0.94 0.94 0.94]);
            yy_out = yy;
        end

        function createPlotLines(app)
            % 用普通 line 代替 animatedline (避免 addpoints 兼容性问题)
            colors = {'r', [0 0.7 0], 'b'};
            app.linePos = gobjects(1, 3);
            app.lineAtt = gobjects(1, 3);
            app.lineCtrl = gobjects(1, 3);
            for i = 1:3
                app.linePos(i)  = line(app.axPos, NaN, NaN, 'Color', colors{i}, 'LineWidth', 1);
                app.lineAtt(i)  = line(app.axAtt, NaN, NaN, 'Color', colors{i}, 'LineWidth', 1);
                app.lineCtrl(i) = line(app.axCtrl, NaN, NaN, 'Color', colors{i}, 'LineWidth', 1);
            end
            legend(app.axPos, 'x','y','z', 'Location','northwest', 'FontSize',7);
            legend(app.axAtt, 'roll','pitch','yaw', 'Location','northwest', 'FontSize',7);
            legend(app.axCtrl, 'T(N)','\alpha(deg)','\beta(deg)', 'Location','northwest', 'FontSize',7);
        end
    end

    %% ===== 数据记录 + 可视化 (private) =====
    methods (Access = private)
        function readUIParams(app)
            att_kp_val = get(app.sliderAttKp, 'Value');
            app.p.att_Kp = [att_kp_val; att_kp_val; att_kp_val * 0.5];

            rate_kp_val = get(app.sliderRateKp, 'Value');
            app.p.rate_Kp = [rate_kp_val; rate_kp_val; rate_kp_val * 0.625];

            pos_kp_val = get(app.sliderPosKp, 'Value');
            app.p.pos_Kp = [pos_kp_val; pos_kp_val; pos_kp_val * 1.5];

            pos_kd_val = get(app.sliderPosKd, 'Value');
            app.p.pos_Kd = [pos_kd_val; pos_kd_val; pos_kd_val * 1.2];
        end

        function recordData(app)
            % 往环形缓冲区写入当前帧数据
            idx = mod(app.buf_idx, app.buf_max) + 1;
            app.buf_t(idx)      = app.t_sim;
            app.buf_pos(idx, :) = app.x(1:3)';
            [roll, pitch, yaw]  = quat2euler(app.x(7:10));
            app.buf_att(idx, :) = rad2deg([roll, pitch, yaw]);
            app.buf_ctrl(idx,:) = [app.last_u(1), rad2deg(app.last_u(2)), rad2deg(app.last_u(3))];
            app.buf_idx = idx;

            % 轨迹缓冲
            tidx = mod(app.trail_idx, size(app.trail_buf, 1)) + 1;
            app.trail_buf(tidx, :) = app.x(1:3)';
            app.trail_idx = tidx;
        end

        function updateVisuals(app)
            pos = app.x(1:3);
            q   = app.x(7:10);
            alpha_g = app.last_u(2);
            beta_g  = app.last_u(3);
            T_val   = app.last_u(1);

            set(app.model.root_tf, 'Matrix', makehgtform('translate', pos'));

            R = quat2rotm(q);
            M_body = eye(4); M_body(1:3,1:3) = R;
            set(app.model.body_tf, 'Matrix', M_body);

            ca = cos(alpha_g); sa = sin(alpha_g);
            cb = cos(-beta_g); sb = sin(-beta_g);
            Ry = [ca 0 sa; 0 1 0; -sa 0 ca];
            Rx = [1 0 0; 0 cb -sb; 0 sb cb];
            M_gim = eye(4); M_gim(1:3,1:3) = Rx * Ry;
            set(app.model.gimbal_tf, 'Matrix', M_gim);

            arrow_len = 0.1 + 0.3 * T_val / (app.p.m * app.p.g);
            set(app.model.thrust_arrow, 'ZData', [0, arrow_len]);
            hb = arrow_len - 0.02; ht = arrow_len + 0.02;
            set(app.model.thrust_head, 'Vertices', ...
                [0.015 0 hb; -0.015 0 hb; 0 0 ht; 0 0.015 hb; 0 -0.015 hb; 0 0 ht]);

            % 轨迹线 (line 对象)
            n = min(app.trail_idx, size(app.trail_buf, 1));
            if n > 0
                if app.trail_idx <= size(app.trail_buf, 1)
                    seg = app.trail_buf(1:n, :);
                else
                    idx_start = mod(app.trail_idx, size(app.trail_buf,1)) + 1;
                    seg = [app.trail_buf(idx_start:end, :); app.trail_buf(1:mod(app.trail_idx, size(app.trail_buf,1)), :)];
                end
                set(app.model.trail, 'XData', seg(:,1), 'YData', seg(:,2), 'ZData', seg(:,3));
            end

            % 目标标记
            set(app.model.target, ...
                'XData', get(app.sliderTargetX, 'Value'), ...
                'YData', get(app.sliderTargetY, 'Value'), ...
                'ZData', get(app.sliderTargetZ, 'Value'));
        end

        function updatePlots(app)
            % 获取有效数据范围
            n = min(app.buf_idx, app.buf_max);
            if n == 0
                % 无数据，清空曲线
                for i = 1:3
                    set(app.linePos(i), 'XData', NaN, 'YData', NaN);
                    set(app.lineAtt(i), 'XData', NaN, 'YData', NaN);
                    set(app.lineCtrl(i), 'XData', NaN, 'YData', NaN);
                end
                return;
            end

            % 从环形缓冲区取出有序数据
            if app.buf_idx <= app.buf_max
                tt  = app.buf_t(1:n);
                pp  = app.buf_pos(1:n, :);
                aa  = app.buf_att(1:n, :);
                cc  = app.buf_ctrl(1:n, :);
            else
                idx_start = mod(app.buf_idx, app.buf_max) + 1;
                order = [idx_start:app.buf_max, 1:mod(app.buf_idx, app.buf_max)];
                tt  = app.buf_t(order);
                pp  = app.buf_pos(order, :);
                aa  = app.buf_att(order, :);
                cc  = app.buf_ctrl(order, :);
            end

            % 更新 line XData/YData
            for i = 1:3
                set(app.linePos(i),  'XData', tt, 'YData', pp(:,i));
                set(app.lineAtt(i),  'XData', tt, 'YData', aa(:,i));
                set(app.lineCtrl(i), 'XData', tt, 'YData', cc(:,i));
            end

            % 滚动窗口
            t = app.t_sim;
            t_lo = max(0, t - app.plot_window);
            t_hi = max(app.plot_window, t + 1);
            xlim(app.axPos,  [t_lo, t_hi]);
            xlim(app.axAtt,  [t_lo, t_hi]);
            xlim(app.axCtrl, [t_lo, t_hi]);
        end

        function updateInfoPanel(app, pos_des)
            pos = app.x(1:3);
            [roll, pitch, yaw] = quat2euler(app.x(7:10));
            pos_err = norm(pos_des - pos);

            info_lines = { ...
                sprintf('Pos err: %.3f m', pos_err), ...
                sprintf('Att: R=%.1f P=%.1f Y=%.1f deg', ...
                    rad2deg(roll), rad2deg(pitch), rad2deg(yaw)), ...
                sprintf('Thrust: %.1f N', app.last_u(1)), ...
                sprintf('Gimbal: a=%.1f b=%.1f deg', ...
                    rad2deg(app.last_u(2)), rad2deg(app.last_u(3)))};
            set(app.lblInfo, 'String', info_lines);
            set(app.lblTime, 'String', sprintf('t = %.1f s', app.t_sim));
        end

        function updateSliderLabels(app)
            set(app.lblTargetX, 'String', sprintf('%.1f m', get(app.sliderTargetX, 'Value')));
            set(app.lblTargetY, 'String', sprintf('%.1f m', get(app.sliderTargetY, 'Value')));
            set(app.lblTargetZ, 'String', sprintf('%.1f m', get(app.sliderTargetZ, 'Value')));
            set(app.lblDistFx, 'String', sprintf('%.1f N', get(app.sliderDistFx, 'Value')));
            set(app.lblDistFy, 'String', sprintf('%.1f N', get(app.sliderDistFy, 'Value')));
            set(app.lblAttKp, 'String', sprintf('%.2f', get(app.sliderAttKp, 'Value')));
            set(app.lblRateKp, 'String', sprintf('%.3f', get(app.sliderRateKp, 'Value')));
            set(app.lblPosKp, 'String', sprintf('%.2f', get(app.sliderPosKp, 'Value')));
            set(app.lblPosKd, 'String', sprintf('%.2f', get(app.sliderPosKd, 'Value')));
        end
    end
end
