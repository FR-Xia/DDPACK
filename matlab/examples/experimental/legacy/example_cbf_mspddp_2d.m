%% 2D CBF-MSPDDP Trajectory Optimization Example / 二维CBF-MSPDDP轨迹优化示例
% Demonstrates how to use UAVTO toolbox for 2D Control Barrier Function Model Predictive Differential Dynamic Programming / 演示如何使用UAVTO工具箱进行二维控制屏障函数模型预测差分动态规划

clear; clc; close all;

fprintf('=== 2D CBF-MSPDDP Trajectory Optimization Example ===\n\n');

try
    % Create optimizer using the same parameters as main_2d_cbf_mspddp.m / 使用与main_2d_cbf_mspddp.m相同的参数创建优化器
    fprintf('Creating CBF-MSPDDP optimizer...\n');
    
    % Parameter settings strictly consistent with main_2d_cbf_mspddp.m / 与main_2d_cbf_mspddp.m严格一致的参数设置
    s_p = 5;  % Scaling parameter / 缩放参数
    x_0 = [0; 0; 90*pi/180];  % Initial state / 初始状态
    x_mid = [50.0/s_p; 50.0/s_p; 30*pi/180];  % Intermediate state / 中间状态
    x_star = [100.0/s_p; 100.0/s_p; 90*pi/180];  % Target state / 目标状态
    t_f = 15.0;  % Final time / 最终时间
    dt = 0.1;  % Time step / 时间步长
    u_max = pi/6;  % Maximum control input / 最大控制输入
    v = 1.0;  % Velocity / 速度
    Q_f = [10.0   0.0     0.0;
           0.0    10.0    0.0;
           0.0    0.0     10.0];  % Final state weight (consistent with main_2d_cbf_mspddp.m) / 最终状态权重（与main_2d_cbf_mspddp.m一致）
    Q_s = [0.0    0.0     0.0;
           0.0    0.0     0.0;
           0.0    0.0     0.0];  % Running state weight (consistent with main_2d_cbf_mspddp.m) / 运行状态权重（与main_2d_cbf_mspddp.m一致）
    R = 0.007;  % Control weight (consistent with main_2d_cbf_mspddp.m) / 控制权重（与main_2d_cbf_mspddp.m一致）
    num_iter = 300;  % Number of iterations / 迭代次数
    
    % CBF-MSPDDP algorithm parameters (consistent with main_2d_cbf_mspddp.m) / CBF-MSPDDP算法参数（与main_2d_cbf_mspddp.m一致）
    rho = 1;  % Penalty parameter / 惩罚参数
    update_epsilon = 0.9;  % Update parameter / 更新参数
    epsilon = 0.4;  % Line search parameter (alpha_l) / 线搜索参数（alpha_l）
    
    % Obstacle settings (single_obs = 0 for multiple obstacles) / 障碍物设置（single_obs = 0表示多个障碍物）
    single_obs = 0;  % Multiple obstacles mode / 多障碍物模式
    obs_position = [24/s_p, 40/s_p, 74/s_p; 
                    25/s_p, 55/s_p, 74/s_p];
    obs_radius = [9/s_p, 9/s_p, 9/s_p];
    num_obs = size(obs_position, 2);  % Number of obstacles / 障碍物数量
    gamma_h = 7;  % Safety distance parameter / 安全距离参数
    
    % Create optimizer and set parameters / 创建优化器并设置参数
    optimizer = UAVTO('algorithm', 'cbf_mspddp_2d');
    optimizer.x0 = x_0;
    optimizer.xf = x_mid;  % First go to intermediate point / 首先到达中间点
    optimizer.tf = t_f;
    optimizer.dt = dt;
    optimizer.u_max = u_max;
    optimizer.v = v;
    optimizer.Q_f = Q_f;
    optimizer.Q_s = Q_s;
    optimizer.R = R;
    optimizer.num_iter = num_iter;
    optimizer.obs_position = obs_position;
    optimizer.obs_radius = obs_radius;
    optimizer.gamma_h = gamma_h;
    optimizer.rho = rho;
    optimizer.update_epsilon = update_epsilon;
    optimizer.epsilon = epsilon;
    optimizer.single_obs = single_obs;
    optimizer.num_obs = num_obs;
    
    % Display parameters / 显示参数
    fprintf('\nParameter settings (consistent with main_2d_cbf_mspddp.m):\n');
    fprintf('  Scaling parameter s_p: %.0f\n', s_p);
    fprintf('  Initial state: [%.2f, %.2f, %.2f]\n', optimizer.x0(1), optimizer.x0(2), optimizer.x0(3));
    fprintf('  Intermediate state: [%.2f, %.2f, %.2f]\n', x_mid(1), x_mid(2), x_mid(3));
    fprintf('  Target state: [%.2f, %.2f, %.2f]\n', x_star(1), x_star(2), x_star(3));
    fprintf('  Final time: %.1f s\n', optimizer.tf);
    fprintf('  Time step: %.2f s\n', optimizer.dt);
    fprintf('  Number of iterations: %d\n', optimizer.num_iter);
    fprintf('  Maximum control: %.4f rad/s\n', optimizer.u_max);
    fprintf('  Velocity: %.1f m/s\n', optimizer.v);
    fprintf('  Safety distance parameter: %.1f\n', optimizer.gamma_h);
    fprintf('  Penalty parameter rho: %.1f\n', optimizer.rho);
    fprintf('  Update parameter update_epsilon: %.1f\n', optimizer.update_epsilon);
    fprintf('  Line search parameter epsilon: %.1f\n', optimizer.epsilon);
    fprintf('  Obstacle mode: Multiple obstacles (single_obs = %d)\n', optimizer.single_obs);
    fprintf('  Number of obstacles: %d\n', optimizer.num_obs);
    
    % Display cost function weights / 显示代价函数权重
    fprintf('\nCost function weights:\n');
    fprintf('  Q_f (Final state weight):\n');
    disp(optimizer.Q_f);
    fprintf('  Q_s (Running state weight):\n');
    disp(optimizer.Q_s);
    fprintf('  R (Control weight): %.3f\n', optimizer.R);
    
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
                sol2 = solution2.sol;  % Get the actual CBF-MSPDDP results / 获取实际的CBF-MSPDDP结果
                fprintf('  Number of trajectory points: %d\n', length(sol2.t));
                fprintf('  Final cost: %.6f\n', sol2.J(end));
                if isfield(sol2, 'iter')
                    fprintf('  Convergence iterations: %d\n', sol2.iter);
                else
                    fprintf('  Convergence iterations: Not recorded\n');
                end
            else
                % Direct CBF-MSPDDP returned solution structure / 直接CBF-MSPDDP返回的解结构
                sol2 = solution2;
                fprintf('  Number of trajectory points: %d\n', length(sol2.t));
                fprintf('  Final cost: %.6f\n', sol2.J(end));
                if isfield(sol2, 'iter')
                    fprintf('  Convergence iterations: %d\n', sol2.iter);
                else
                    fprintf('  Convergence iterations: Not recorded\n');
                end
            end
            
            
            % Plot results - only show one figure / 绘制结果 - 只显示一个图形
            fprintf('\nPlotting results...\n');
            figure(1);
            set(gcf, 'Name', 'CBF-MSPDDP 2D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
            
            % Plot CBF-MSPDDP results / 绘制CBF-MSPDDP结果
            plot_cbf_mspddp_detailed_analysis(sol1, sol2, optimizer, obs_position, obs_radius, s_p, x_0, x_mid, x_star);
            
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
fprintf('Use help UAVTO for more help information.\n');

%% Custom plotting function / 自定义绘图函数

function plot_cbf_mspddp_detailed_analysis(sol1, sol2, optimizer, obs_position, obs_radius, s_p, x_0, x_mid, x_star)
    % Plot CBF-MSPDDP simplified analysis - only show trajectory, cost function, energy / 绘制CBF-MSPDDP简化分析 - 只显示轨迹、代价函数、能量
    
    % Debug information: check solution structure / 调试信息：检查解结构
    fprintf('Debug information:\n');
    fprintf('  sol1.x type: %s\n', class(sol1.x));
    fprintf('  sol2.x type: %s\n', class(sol2.x));
    if iscell(sol1.x)
        fprintf('  sol1.x length: %d\n', length(sol1.x));
        fprintf('  sol2.x length: %d\n', length(sol2.x));
    end
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(sol1.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj1 = zeros(3, length(sol1.x));
        for i = 1:length(sol1.x)
            x_traj1(:, i) = sol1.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj1 = sol1.x;
    end
    
    if iscell(sol2.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj2 = zeros(3, length(sol2.x));
        for i = 1:length(sol2.x)
            x_traj2(:, i) = sol2.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj2 = sol2.x;
    end
    
    fprintf('  First segment trajectory points: %d\n', size(x_traj1, 2));
    fprintf('  Second segment trajectory points: %d\n', size(x_traj2, 2));
    fprintf('  First segment trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Theta[%.4f, %.4f]\n', ...
        min(x_traj1(1,:)), max(x_traj1(1,:)), min(x_traj1(2,:)), max(x_traj1(2,:)), ...
        min(x_traj1(3,:)), max(x_traj1(3,:)));
    fprintf('  Second segment trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Theta[%.4f, %.4f]\n', ...
        min(x_traj2(1,:)), max(x_traj2(1,:)), min(x_traj2(2,:)), max(x_traj2(2,:)), ...
        min(x_traj2(3,:)), max(x_traj2(3,:)));
    
    % Trajectory and obstacles (Figure 1) / 轨迹和障碍物（图1）
    subplot(1, 3, 1);
    hold on;
    
    % Plot obstacles / 绘制障碍物
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(obs_position, 2)
        x_circle = obs_position(1, i) + obs_radius(i) * cos(theta);
        y_circle = obs_position(2, i) + obs_radius(i) * sin(theta);
        patch(x_circle*s_p, y_circle*s_p, 'r', 'EdgeColor', 'k', 'LineWidth', 1);
    end
    
    % Plot trajectory - use different colors / 绘制轨迹 - 使用不同颜色
    h_traj1 = plot(x_traj1(1, :)*s_p, x_traj1(2, :)*s_p, 'b-', 'LineWidth', 2);
    h_traj2 = plot(x_traj2(1, :)*s_p, x_traj2(2, :)*s_p, 'r-', 'LineWidth', 2);
    
    % Plot start point, waypoint, end point / 绘制起点、航点、终点
    h_start = plot(x_0(1)*s_p, x_0(2)*s_p, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h_waypoint = plot(x_mid(1)*s_p, x_mid(2)*s_p, 'mo', 'MarkerSize', 10, 'LineWidth', 2);
    h_end = plot(x_star(1)*s_p, x_star(2)*s_p, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); title('Trajectory'); grid on; axis equal;
    legend([h_traj1, h_traj2, h_start, h_waypoint, h_end], ...
           'Segment 1', 'Segment 2', 'Start', 'Waypoint', 'End', 'Location', 'best');
    
    % Set appropriate axis range / 设置适当的坐标轴范围
    x_min = min([x_traj1(1, :), x_traj2(1, :), x_0(1), x_mid(1), x_star(1)]) * s_p;
    x_max = max([x_traj1(1, :), x_traj2(1, :), x_0(1), x_mid(1), x_star(1)]) * s_p;
    y_min = min([x_traj1(2, :), x_traj2(2, :), x_0(2), x_mid(2), x_star(2)]) * s_p;
    y_max = max([x_traj1(2, :), x_traj2(2, :), x_0(2), x_mid(2), x_star(2)]) * s_p;
    
    x_margin = (x_max - x_min) * 0.1;
    y_margin = (y_max - y_min) * 0.1;
    if x_margin == 0, x_margin = 1; end
    if y_margin == 0, y_margin = 1; end
    
    xlim([x_min - x_margin, x_max + x_margin]);
    ylim([y_min - y_margin, y_max + y_margin]);
    
    % Cost function convergence (Figure 2) / 代价函数收敛（图2）
    subplot(1, 3, 2);
    plot(sol1.J, 'b-', 'LineWidth', 2);
    hold on;
    plot(sol2.J, 'r-', 'LineWidth', 2);
    xlabel('Iterations'); ylabel('Cost Function Value'); title('Cost Function Convergence'); grid on;
    legend('Segment 1', 'Segment 2', 'Location', 'best');
    
    % Control energy convergence (Figure 3) / 控制能量收敛（图3）
    subplot(1, 3, 3);
    plot(sol1.E, 'b-', 'LineWidth', 2);
    hold on;
    plot(sol2.E, 'r-', 'LineWidth', 2);
    xlabel('Iterations'); ylabel('Control Energy'); title('Control Energy Convergence'); grid on;
    legend('Segment 1', 'Segment 2', 'Location', 'best');
end
