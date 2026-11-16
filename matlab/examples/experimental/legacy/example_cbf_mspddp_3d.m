%% 3D CBF-MSPDDP Trajectory Optimization Example / 三维CBF-MSPDDP轨迹优化示例
% Demonstrates how to use UAVTO toolbox for 3D Control Barrier Function Model Predictive Differential Dynamic Programming / 演示如何使用UAVTO工具箱进行三维控制屏障函数模型预测差分动态规划

clear; clc; close all;

fprintf('=== 3D CBF-MSPDDP Trajectory Optimization Example ===\n\n');

try
    % Create optimizer using the same parameters as main_3d_cbf_mspddp.m / 使用与main_3d_cbf_mspddp.m相同的参数创建优化器
    fprintf('Creating CBF-MSPDDP optimizer...\n');
    
    % Parameter settings strictly consistent with main_3d_cbf_mspddp.m / 与main_3d_cbf_mspddp.m严格一致的参数设置
    s_p = 10;  % Scaling parameter / 缩放参数
    x_0 = [200*s_p; 200*s_p; 200*s_p; 5*s_p*s_p; -45/180*pi; 240/180*pi];  % Initial state / 初始状态
    x_mid = [100*s_p; 100*s_p; 100*s_p; 5*s_p*s_p; -60/180*pi; 180/180*pi];  % Intermediate state / 中间状态
    x_star = [0.0; 0.0; 0.0; 5*s_p*s_p; -70/180*pi; 240/180*pi];  % Target state / 目标状态
    t_f = 35.0;  % Final time / 最终时间
    dt = 0.1;  % Time step / 时间步长
    u_max = [30; 30];  % Maximum control input / 最大控制输入
    m = 5;  % Mass / 质量
    g = 9.81;  % Gravity acceleration / 重力加速度
    Ixx = 0.05;  % Moment of inertia / 转动惯量
    Iyy = 0.05;  % Moment of inertia / 转动惯量
    Izz = 0.1;   % Moment of inertia / 转动惯量
    Q_f = blkdiag(2000*s_p*eye(3), 100/(s_p)^4*eye(1), 5000*eye(2));  % Final state weight / 最终状态权重
    Q_s = blkdiag(0.0000001*s_p*eye(3), 0.01*eye(1), 0.0000001*eye(2));  % Running state weight / 运行状态权重
    R = 1 * eye(numel(u_max));  % Control weight / 控制权重
    num_iter = 300;  % Number of iterations / 迭代次数
    
    % CBF-MSPDDP algorithm parameters (consistent with main_3d_cbf_mspddp.m) / CBF-MSPDDP算法参数（与main_3d_cbf_mspddp.m一致）
    epsilon = 0.5;  % Line search parameter (alpha_l) / 线搜索参数（alpha_l）
    gamma_h_1 = 50;  % Safety distance parameter 1 / 安全距离参数1
    gamma_h_2 = 50;  % Safety distance parameter 2 / 安全距离参数2
    rho = 1;  % Penalty parameter / 惩罚参数
    update_epsilon = 0.9;  % Update parameter / 更新参数
    
    % Obstacle settings (consistent with main_3d_cbf_mspddp.m) / 障碍物设置（与main_3d_cbf_mspddp.m一致）
    single_obs = 1;  % Single obstacle mode / 单障碍物模式
    obs_position_1 = [145*s_p; 150*s_p];  % Obstacle 1 position / 障碍物1位置
    obs_radius_1 = 10*s_p;  % Obstacle 1 radius / 障碍物1半径
    obs_position_2 = [50*s_p; 50*s_p];   % Obstacle 2 position / 障碍物2位置
    obs_radius_2 = 10*s_p;  % Obstacle 2 radius / 障碍物2半径
    num_obs = 1;  % Number of obstacles / 障碍物数量
    
    % Create optimizer and set parameters / 创建优化器并设置参数
    optimizer = DDPACK('algorithm', 'cbf_mspddp_3d', 'dimension', 3);
    optimizer.x0 = x_0;
    optimizer.xf = x_mid;  % First go to intermediate point / 首先到达中间点
    optimizer.tf = t_f;
    optimizer.dt = dt;
    optimizer.u_max = u_max;
    optimizer.m = m;
    optimizer.g = g;
    optimizer.s_p = s_p;
    optimizer.Q_f = Q_f;
    optimizer.Q_s = Q_s;
    optimizer.R = R;
    optimizer.num_iter = num_iter;
    optimizer.obs_position = obs_position_1;
    optimizer.obs_radius = obs_radius_1;
    optimizer.gamma_h = gamma_h_1;
    optimizer.rho = rho;
    optimizer.update_epsilon = update_epsilon;
    optimizer.epsilon = epsilon;
    optimizer.single_obs = single_obs;
    optimizer.num_obs = num_obs;
    
    % Display parameters / 显示参数
    fprintf('\nParameter settings (consistent with main_3d_cbf_mspddp.m):\n');
    fprintf('  Scaling parameter s_p: %.0f\n', s_p);
    fprintf('  Initial state: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
        optimizer.x0(1), optimizer.x0(2), optimizer.x0(3), optimizer.x0(4), optimizer.x0(5), optimizer.x0(6));
    fprintf('  Intermediate state: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
        x_mid(1), x_mid(2), x_mid(3), x_mid(4), x_mid(5), x_mid(6));
    fprintf('  Target state: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
        x_star(1), x_star(2), x_star(3), x_star(4), x_star(5), x_star(6));
    fprintf('  Final time: %.1f s\n', optimizer.tf);
    fprintf('  Time step: %.2f s\n', optimizer.dt);
    fprintf('  Number of iterations: %d\n', optimizer.num_iter);
    fprintf('  Maximum control: [%.1f, %.1f]\n', optimizer.u_max(1), optimizer.u_max(2));
    fprintf('  Mass: %.1f kg\n', optimizer.m);
    fprintf('  Gravity acceleration: %.2f m/s²\n', optimizer.g);
    fprintf('  Safety distance parameter 1: %.1f\n', optimizer.gamma_h);
    fprintf('  Safety distance parameter 2: %.1f\n', gamma_h_2);
    fprintf('  Penalty parameter rho: %.1f\n', optimizer.rho);
    fprintf('  Update parameter update_epsilon: %.1f\n', optimizer.update_epsilon);
    fprintf('  Line search parameter epsilon: %.1f\n', optimizer.epsilon);
    fprintf('  Obstacle mode: Single obstacle (single_obs = %d)\n', optimizer.single_obs);
    fprintf('  Number of obstacles: %d\n', optimizer.num_obs);
    
    % Display cost function weights / 显示代价函数权重
    fprintf('\nCost function weights:\n');
    fprintf('  Q_f (Final state weight):\n');
    disp(optimizer.Q_f);
    fprintf('  Q_s (Running state weight):\n');
    disp(optimizer.Q_s);
    fprintf('  R (Control weight):\n');
    disp(optimizer.R);
    
    % Solve first segment / 求解第一段
    fprintf('\nStarting to solve first segment trajectory...\n');
    tic;
    solution1 = optimizer.solve();
    solve_time1 = toc;
    
    if ~isempty(solution1)
        fprintf('✓ First segment solved successfully!\n');
        fprintf('  Solve time: %.2f seconds\n', solve_time1);
        
        % Check solution structure fields / 检查解结构字段
        if isfield(solution1, 'sol') && isfield(solution1, 'algorithm')
            % UAVTO class returned solution structure containing CBF-MSPDDP results / UAVTO类返回包含CBF-MSPDDP结果的解结构
            sol1 = solution1.sol;  % Get the actual CBF-MSPDDP results / 获取实际的CBF-MSPDDP结果
            fprintf('  Number of trajectory points: %d\n', length(sol1.t));
            fprintf('  Final cost: %.6f\n', sol1.J(end));
            if isfield(sol1, 'iter')
                fprintf('  Convergence iterations: %d\n', sol1.iter);
            else
                fprintf('  Convergence iterations: Not recorded\n');
            end
        else
            % Direct CBF-MSPDDP returned solution structure / 直接CBF-MSPDDP返回的解结构
            sol1 = solution1;
            fprintf('  Number of trajectory points: %d\n', length(sol1.t));
            fprintf('  Final cost: %.6f\n', sol1.J(end));
            if isfield(sol1, 'iter')
                fprintf('  Convergence iterations: %d\n', sol1.iter);
            else
                fprintf('  Convergence iterations: Not recorded\n');
            end
        end
        
        % Set second segment target / 设置第二段目标
        optimizer.x0 = x_mid;
        optimizer.xf = x_star;
        optimizer.obs_position = obs_position_2;
        optimizer.obs_radius = obs_radius_2;
        optimizer.gamma_h = gamma_h_2;
        
        % Solve second segment / 求解第二段
        fprintf('\nStarting to solve second segment trajectory...\n');
        tic;
        solution2 = optimizer.solve();
        solve_time2 = toc;
        
        if ~isempty(solution2)
            fprintf('✓ Second segment solved successfully!\n');
            fprintf('  Solve time: %.2f seconds\n', solve_time2);
            
            % Check solution structure fields / 检查解结构字段
            if isfield(solution2, 'sol') && isfield(solution2, 'algorithm')
                % UAVTO class returned solution structure containing CBF-MSPDDP results / UAVTO类返回包含CBF-MSPDDP结果的解结构
                sol1_2 = solution2.sol;  % Get the actual CBF-MSPDDP results / 获取实际的CBF-MSPDDP结果
                fprintf('  Number of trajectory points: %d\n', length(sol1_2.t));
                fprintf('  Final cost: %.6f\n', sol1_2.J(end));
                if isfield(sol1_2, 'iter')
                    fprintf('  Convergence iterations: %d\n', sol1_2.iter);
                else
                    fprintf('  Convergence iterations: Not recorded\n');
                end
            else
                % Direct CBF-MSPDDP returned solution structure / 直接CBF-MSPDDP返回的解结构
                sol1_2 = solution2;
                fprintf('  Number of trajectory points: %d\n', length(sol1_2.t));
                fprintf('  Final cost: %.6f\n', sol1_2.J(end));
                if isfield(sol1_2, 'iter')
                    fprintf('  Convergence iterations: %d\n', sol1_2.iter);
                else
                    fprintf('  Convergence iterations: Not recorded\n');
                end
            end
            
            % Result analysis / 结果分析
            fprintf('\nResult Analysis:\n');
            
            % Final point error / 最终点误差
            final_error1 = norm(sol1.x{end} - x_mid);
            final_error2 = norm(sol1_2.x{end} - x_star);
            fprintf('  First segment final point error: %.6f\n', final_error1);
            fprintf('  Second segment final point error: %.6f\n', final_error2);
            
            % Control input statistics / 控制输入统计
            u_values1 = cell2mat(sol1.u);
            u_values2 = cell2mat(sol1_2.u);
            max_control1 = max(abs(u_values1), [], 2);
            max_control2 = max(abs(u_values2), [], 2);
            mean_control1 = mean(abs(u_values1), 2);
            mean_control2 = mean(abs(u_values2), 2);
            fprintf('  First segment maximum control input: [%.4f, %.4f]\n', max_control1(1), max_control1(2));
            fprintf('  First segment average control input: [%.4f, %.4f]\n', mean_control1(1), mean_control1(2));
            fprintf('  Second segment maximum control input: [%.4f, %.4f]\n', max_control2(1), max_control2(2));
            fprintf('  Second segment average control input: [%.4f, %.4f]\n', mean_control2(1), mean_control2(2));
            
            % Trajectory length / 轨迹长度
            trajectory_length1 = 0;
            for i = 2:length(sol1.x)
                dx = sol1.x{i}(1) - sol1.x{i-1}(1);
                dy = sol1.x{i}(2) - sol1.x{i-1}(2);
                dz = sol1.x{i}(3) - sol1.x{i-1}(3);
                trajectory_length1 = trajectory_length1 + sqrt(dx^2 + dy^2 + dz^2);
            end
            
            trajectory_length2 = 0;
            for i = 2:length(sol1_2.x)
                dx = sol1_2.x{i}(1) - sol1_2.x{i-1}(1);
                dy = sol1_2.x{i}(2) - sol1_2.x{i-1}(2);
                dz = sol1_2.x{i}(3) - sol1_2.x{i-1}(3);
                trajectory_length2 = trajectory_length2 + sqrt(dx^2 + dy^2 + dz^2);
            end
            fprintf('  First segment trajectory length: %.4f (scaling units)\n', trajectory_length1);
            fprintf('  Second segment trajectory length: %.4f (scaling units)\n', trajectory_length2);
            
            
            % Plot results - only show one figure / 绘制结果 - 只显示一个图形
            fprintf('\nPlotting results...\n');
            figure(1);
            set(gcf, 'Name', 'CBF-MSPDDP 3D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
            
            % Plot CBF-MSPDDP results / 绘制CBF-MSPDDP结果
            plot_cbf_mspddp_3d_detailed_analysis(sol1, sol1_2, optimizer, obs_position_1, obs_radius_1, obs_position_2, obs_radius_2, s_p, x_0, x_mid, x_star);
            
        else
            fprintf('✗ Second segment solving failed\n');
        end
        
    else
        fprintf('✗ First segment solving failed\n');
    end
    
catch ME
    fprintf('✗ Example execution failed: %s\n', ME.message);
    fprintf('Error location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    rethrow(ME);
end

fprintf('\nExample completed!\n');
fprintf('Use help DDPACK for more help information.\n');

%% Custom plotting functions / 自定义绘图函数

function plot_cbf_mspddp_3d_detailed_analysis(sol1, sol1_2, optimizer, obs_position_1, obs_radius_1, obs_position_2, obs_radius_2, s_p, x_0, x_mid, x_star)
    % Plot CBF-MSPDDP 3D simplified analysis - only display Trajectory, cost function, energy / 绘制CBF-MSPDDP三维简化分析 - 只显示轨迹、代价函数、能量
    
    % Debug information: check solution structure / 调试信息：检查解结构
    fprintf('Debug information:\n');
    fprintf('  sol1.xtype: %s\n', class(sol1.x));
    fprintf('  sol1_2.xtype: %s\n', class(sol1_2.x));
    if iscell(sol1.x)
        fprintf('  sol1.xlength: %d\n', length(sol1.x));
        fprintf('  sol1_2.xlength: %d\n', length(sol1_2.x));
    end
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(sol1.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj1 = zeros(6, length(sol1.x));
        for i = 1:length(sol1.x)
            x_traj1(:, i) = sol1.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj1 = sol1.x;
    end
    
    if iscell(sol1_2.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj2 = zeros(6, length(sol1_2.x));
        for i = 1:length(sol1_2.x)
            x_traj2(:, i) = sol1_2.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj2 = sol1_2.x;
    end
    
    fprintf('  First segment number of waypoints: %d\n', size(x_traj1, 2));
    fprintf('  Second segment number of waypoints: %d\n', size(x_traj2, 2));
    fprintf('  First segment trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Z[%.4f, %.4f]\n', ...
        min(x_traj1(1,:)), max(x_traj1(1,:)), min(x_traj1(2,:)), max(x_traj1(2,:)), ...
        min(x_traj1(3,:)), max(x_traj1(3,:)));
    fprintf('  Second segment trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Z[%.4f, %.4f]\n', ...
        min(x_traj2(1,:)), max(x_traj2(1,:)), min(x_traj2(2,:)), max(x_traj2(2,:)), ...
        min(x_traj2(3,:)), max(x_traj2(3,:)));
    
    % 3D trajectory and obstacles (Figure 1) / 三维轨迹和障碍物（图1）
    subplot(1, 3, 1);
    hold on;
    
    % Plot obstacles / 绘制障碍物
    h_obstacles = [];
    
    % Obstacle 1 / 障碍物1
    [hCylinder1, hEndPlate1_1, hEndPlate2_1] = cylinder3([obs_position_1(1)/s_p, obs_position_1(2)/s_p, x_0(3)/s_p], ...
        [obs_position_1(1)/s_p, obs_position_1(2)/s_p, x_star(3)/s_p], obs_radius_1/s_p, 1000, 'r', 1, 0);
    h_obstacles = [h_obstacles, hCylinder1];
    
    % Obstacle 2 / 障碍物2
    [hCylinder2, hEndPlate1_2, hEndPlate2_2] = cylinder3([obs_position_2(1)/s_p, obs_position_2(2)/s_p, x_0(3)/s_p], ...
        [obs_position_2(1)/s_p, obs_position_2(2)/s_p, x_star(3)/s_p], obs_radius_2/s_p, 1000, 'r', 1, 0);
    h_obstacles = [h_obstacles, hCylinder2];
    
    % Plot trajectory - use different colors / 绘制轨迹 - 使用不同颜色
    h1 = plot3(x_traj1(1, :)/s_p, x_traj1(2, :)/s_p, x_traj1(3, :)/s_p, 'b-', 'LineWidth', 2);
    h2 = plot3(x_traj2(1, :)/s_p, x_traj2(2, :)/s_p, x_traj2(3, :)/s_p, 'r-', 'LineWidth', 2);
    
    % Plot start point, waypoint, end point / 绘制起点、航点、终点
    h3 = plot3(x_0(1)/s_p, x_0(2)/s_p, x_0(3)/s_p, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h4 = plot3(x_mid(1)/s_p, x_mid(2)/s_p, x_mid(3)/s_p, 'mo', 'MarkerSize', 8, 'LineWidth', 2);
    h5 = plot3(x_star(1)/s_p, x_star(2)/s_p, x_star(3)/s_p, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('3D Trajectory'); grid on;
    
    % Create correct legend / 创建正确的图例
    if ~isempty(h_obstacles)
        legend([h_obstacles(1), h1, h2, h3, h4, h5], {'Obstacles', 'Segment 1', 'Segment 2', 'Start', 'Waypoint', 'End'}, 'Location', 'best');
    else
        legend([h1, h2, h3, h4, h5], {'Segment 1', 'Segment 2', 'Start', 'Waypoint', 'End'}, 'Location', 'best');
    end
    
    % Set appropriate axis range / 设置适当的坐标轴范围
    x_min = min([x_traj1(1, :), x_traj2(1, :), x_0(1), x_mid(1), x_star(1)]) / s_p;
    x_max = max([x_traj1(1, :), x_traj2(1, :), x_0(1), x_mid(1), x_star(1)]) / s_p;
    y_min = min([x_traj1(2, :), x_traj2(2, :), x_0(2), x_mid(2), x_star(2)]) / s_p;
    y_max = max([x_traj1(2, :), x_traj2(2, :), x_0(2), x_mid(2), x_star(2)]) / s_p;
    z_min = min([x_traj1(3, :), x_traj2(3, :), x_0(3), x_mid(3), x_star(3)]) / s_p;
    z_max = max([x_traj1(3, :), x_traj2(3, :), x_0(3), x_mid(3), x_star(3)]) / s_p;
    
    x_margin = (x_max - x_min) * 0.1;
    y_margin = (y_max - y_min) * 0.1;
    z_margin = (z_max - z_min) * 0.1;
    if x_margin == 0, x_margin = 10; end
    if y_margin == 0, y_margin = 10; end
    if z_margin == 0, z_margin = 10; end
    
    xlim([x_min - x_margin, x_max + x_margin]);
    ylim([y_min - y_margin, y_max + y_margin]);
    zlim([z_min - z_margin, z_max + z_margin]);
    
    % Set 3D view angle for better perspective / 设置3D视角以获得更好的透视效果
    view(45, 30);  % Azimuth 45°, Elevation 30° / 方位角45°，仰角30°
    
    % Cost function convergence (Figure 2) - Fixed version / 代价函数收敛（图2）- 修复版本
    subplot(1, 3, 2);
    hold on;
    
    % Check and process cost function data / 检查和处理代价函数数据
    if isfield(sol1, 'J') && ~isempty(sol1.J)
        % Find the number of actual iterations (non-zero values) / 找到实际迭代次数（非零值）
        J1_nonzero = find(sol1.J ~= 0);
        if ~isempty(J1_nonzero)
            plot(J1_nonzero, sol1.J(J1_nonzero), 'b-', 'LineWidth', 2);
        else
            % If all values are 0, plot original data / 如果所有值都是0，绘制原始数据
            plot(1:length(sol1.J), sol1.J, 'b-', 'LineWidth', 2);
        end
    else
        fprintf('Warning: sol1.J field does not exist or is empty\n');
        plot(1, 0, 'b-', 'LineWidth', 2);
    end
    
    if isfield(sol1_2, 'J') && ~isempty(sol1_2.J)
        % Find the number of actual iterations (non-zero values) / 找到实际迭代次数（非零值）
        J2_nonzero = find(sol1_2.J ~= 0);
        if ~isempty(J2_nonzero)
            plot(J2_nonzero, sol1_2.J(J2_nonzero), 'r-', 'LineWidth', 2);
        else
            % If all values are 0, plot original data / 如果所有值都是0，绘制原始数据
            plot(1:length(sol1_2.J), sol1_2.J, 'r-', 'LineWidth', 2);
        end
    else
        fprintf('Warning: sol1_2.J field does not exist or is empty\n');
        plot(1, 0, 'r-', 'LineWidth', 2);
    end
    
    xlabel('Iterations'); ylabel('Cost function value'); title('Cost function convergence'); grid on;
    
    % Control energy convergence (Figure 3) - Fixed version / 控制能量收敛（图3）- 修复版本
    subplot(1, 3, 3);
    hold on;
    
    % Check and process control energy data / 检查和处理控制能量数据
    if isfield(sol1, 'E') && ~isempty(sol1.E)
        % Find the number of actual iterations (non-zero values) / 找到实际迭代次数（非零值）
        E1_nonzero = find(sol1.E ~= 0);
        if ~isempty(E1_nonzero)
            plot(E1_nonzero, sol1.E(E1_nonzero), 'b-', 'LineWidth', 2);
        else
            % If all values are 0, plot original data / 如果所有值都是0，绘制原始数据
            plot(1:length(sol1.E), sol1.E, 'b-', 'LineWidth', 2);
        end
    else
        fprintf('Warning: sol1.E field does not exist or is empty\n');
        plot(1, 0, 'b-', 'LineWidth', 2);
    end
    
    if isfield(sol1_2, 'E') && ~isempty(sol1_2.E)
        % Find the number of actual iterations (non-zero values) / 找到实际迭代次数（非零值）
        E2_nonzero = find(sol1_2.E ~= 0);
        if ~isempty(E2_nonzero)
            plot(E2_nonzero, sol1_2.E(E2_nonzero), 'r-', 'LineWidth', 2);
        else
            % If all values are 0, plot original data / 如果所有值都是0，绘制原始数据
            plot(1:length(sol1_2.E), sol1_2.E, 'r-', 'LineWidth', 2);
        end
    else
        fprintf('Warning: sol1_2.E field does not exist or is empty\n');
        plot(1, 0, 'r-', 'LineWidth', 2);
    end
    
    xlabel('Iterations'); ylabel('Control energy'); title('Control energy convergence'); grid on;
    
    % Add debug information / 添加调试信息
    fprintf('Plot data check:\n');
    if isfield(sol1, 'J')
        fprintf('  sol1.J length: %d, non-zero values count: %d\n', length(sol1.J), sum(sol1.J ~= 0));
    end
    if isfield(sol1_2, 'J')
        fprintf('  sol1_2.J length: %d, non-zero values count: %d\n', length(sol1_2.J), sum(sol1_2.J ~= 0));
    end
    if isfield(sol1, 'E')
        fprintf('  sol1.E length: %d, non-zero values count: %d\n', length(sol1.E), sum(sol1.E ~= 0));
    end
    if isfield(sol1_2, 'E')
        fprintf('  sol1_2.E length: %d, non-zero values count: %d\n', length(sol1_2.E), sum(sol1_2.E ~= 0));
    end
end

%% Auxiliary function: Draw 3D cylinder obstacles / 辅助函数：绘制三维圆柱体障碍物
function [Cylinder, EndPlate1, EndPlate2] = cylinder3(X1,X2,r,n,cyl_color,closed,lines)
    % Calculate cylinder length / 计算圆柱体长度
    length_cyl=norm(X2-X1);
    
    % Create circle in YZ plane / 在YZ平面创建圆
    t=linspace(0,2*pi,n)';
    x2=r*cos(t);
    x3=r*sin(t);
    
    % Create points in X direction / 在X方向创建点
    x1=[0 length_cyl];
    
    % Create cylinder points / 创建圆柱体点
    xx1=repmat(x1,length(x2),1);
    xx2=repmat(x2,1,2);
    xx3=repmat(x3,1,2);
    
    % Draw two circles of closed cylinder / 绘制封闭圆柱体的两个圆
    if closed==1
        hold on
        EndPlate1=fill3(xx1(:,1),xx2(:,1),xx3(:,1),'r');
        EndPlate2=fill3(xx1(:,2),xx2(:,2),xx3(:,2),'r');
    end
    
    % Draw cylinder / 绘制圆柱体
    Cylinder=mesh(xx1,xx2,xx3);
    
    % Define unit vector in X direction / 定义X方向的单位向量
    unit_Vx=[1 0 0];
    
    % Calculate angle between X direction and required direction / 计算X方向与所需方向之间的角度
    angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
    
    % Find rotation axis / 找到旋转轴
    axis_rot=cross([1 0 0],(X2-X1) );
    
    % Rotate cylinder and end plates / 旋转圆柱体和端板
    if angle_X1X2~=0
        rotate(Cylinder,axis_rot,angle_X1X2,[0 0 0])
        if closed==1
            rotate(EndPlate1,axis_rot,angle_X1X2,[0 0 0])
            rotate(EndPlate2,axis_rot,angle_X1X2,[0 0 0])
        end
    end
    
    % Move to correct position / 移动到正确位置
    if closed==1
        set(EndPlate1,'XData',get(EndPlate1,'XData')+X1(1))
        set(EndPlate1,'YData',get(EndPlate1,'YData')+X1(2))
        set(EndPlate1,'ZData',get(EndPlate1,'ZData')+X1(3))
        
        set(EndPlate2,'XData',get(EndPlate2,'XData')+X1(1))
        set(EndPlate2,'YData',get(EndPlate2,'YData')+X1(2))
        set(EndPlate2,'ZData',get(EndPlate2,'ZData')+X1(3))
    end
    set(Cylinder,'XData',get(Cylinder,'XData')+X1(1))
    set(Cylinder,'YData',get(Cylinder,'YData')+X1(2))
    set(Cylinder,'ZData',get(Cylinder,'ZData')+X1(3))
    
    % Set color / 设置颜色
    set(Cylinder,'FaceColor',cyl_color)
    if closed==1
        set([EndPlate1 EndPlate2],'FaceColor',cyl_color)
    else
        EndPlate1=[];
        EndPlate2=[];
    end
    
    set(Cylinder,'FaceAlpha',0.6)
    set(EndPlate1,'FaceAlpha',0.6)
    set(EndPlate2,'FaceAlpha',0.6)
    
    % If need to hide lines / 如果需要隐藏线条
    if lines==0
        set(Cylinder,'EdgeAlpha',0)
    end
end
