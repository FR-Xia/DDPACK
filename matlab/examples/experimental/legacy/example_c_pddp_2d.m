%% 2D C-PDDP Trajectory Optimization Example / 二维C-PDDP轨迹优化示例
% Demonstrates how to use DDPACK package for 2D constrained trajectory optimization / 演示如何使用DDPACK算法包进行二维约束轨迹优化

clear; clc; close all;

fprintf('=== 2D C-PDDP Trajectory Optimization Example ===\n\n');

try
    % Create optimizer using the same parameters as main_2d_c_pddp.m / 使用与main_2d_c_pddp.m相同的参数创建优化器
    fprintf('Creating C-PDDP optimizer...\n');
    
    % Parameter settings strictly consistent with main_2d_c_pddp.m / 与main_2d_c_pddp.m严格一致的参数设置
    s_p = 30;  % Scaling parameter / 缩放参数
    x_0 = [0.0; 0.0; 0.0];  % Initial state / 初始状态
    x_wpts = [[90/s_p; 180/s_p;], [360/s_p; 270/s_p;]];  % Intermediate waypoints / 中间航点
    x_star = [450.0/s_p; 450.0/s_p; 45/180*pi];  % Target state / 目标状态
    t_f = 27.0;  % Final time / 最终时间
    dt = 0.1;  % Time step / 时间步长
    u_max = 30/180*pi;  % Maximum control input / 最大控制输入
    v = 30.0;  % Velocity / 速度
    Q_f = [20.0    0.0     0.0;
           0.0      20.0   0.0;
           0.0      0.0     20.0];  % Final state weight / 最终状态权重
    Q_s = [0.0    0.0     0.0;
           0.0    0.0     0.0;
           0.0    0.0     0.0];  % Running state weight / 运行状态权重
    R = 1.0;  % Control weight / 控制权重
    num_iter = 500;  % Number of iterations / 迭代次数
    
    % Create optimizer and set parameters / 创建优化器并设置参数
    optimizer = DDPACK('algorithm', 'c_pddp');
    optimizer.x0 = x_0;
    optimizer.xf = x_star;
    optimizer.waypoints = x_wpts;
    optimizer.tf = t_f;
    optimizer.dt = dt;
    optimizer.u_max = u_max;
    optimizer.v = v;
    optimizer.Q_f = Q_f;
    optimizer.Q_s = Q_s;
    optimizer.R = R;
    optimizer.num_iter = num_iter;
    
    % Display parameters / 显示参数
    fprintf('\nParameter settings (consistent with main_2d_c_pddp.m):\n');
    fprintf('  Scaling parameter s_p: %.0f\n', s_p);
    fprintf('  Initial state: [%.2f, %.2f, %.2f]\n', optimizer.x0(1), optimizer.x0(2), optimizer.x0(3));
    fprintf('  Target state: [%.2f, %.2f, %.2f]\n', optimizer.xf(1), optimizer.xf(2), optimizer.xf(3));
    fprintf('  Intermediate waypoints: [%.2f, %.2f], [%.2f, %.2f]\n', x_wpts(1,1), x_wpts(2,1), x_wpts(1,2), x_wpts(2,2));
    fprintf('  Final time: %.1f s\n', optimizer.tf);
    fprintf('  Time step: %.2f s\n', optimizer.dt);
    fprintf('  Number of iterations: %d\n', optimizer.num_iter);
    fprintf('  Maximum control: %.4f rad/s\n', optimizer.u_max);
    fprintf('  Velocity: %.1f m/s\n', optimizer.v);
    
    % Display cost function weights / 显示代价函数权重
    fprintf('\nCost function weights:\n');
    fprintf('  Q_f (Final state weight):\n');
    disp(optimizer.Q_f);
    fprintf('  Q_s (Running state weight):\n');
    disp(optimizer.Q_s);
    fprintf('  R (Control weight): %.2f\n', optimizer.R);
    
    % Solving / 求解
    fprintf('\nStarting solving...\n');
    tic;
    solution = optimizer.solve();
    solve_time = toc;
    
    if ~isempty(solution)
        fprintf('✓ Solving successful!\n');
        fprintf('  Solving time: %.2f seconds\n', solve_time);
        
        % Check solution structure fields / 检查解结构字段
        if isfield(solution, 'sol') && isfield(solution, 'algorithm')
            % DDPACK class returns solution structure containing C-PDDP results / DDPACK类返回包含C-PDDP结果的解结构
            sol = solution.sol;  % Get the actual C-PDDP results / 获取实际的C-PDDP结果
            fprintf('  Number of trajectory points: %d\n', length(sol.t));
            fprintf('  Final cost: %.6f\n', sol.J(end));
            fprintf('  Convergence iterations: %d\n', sol.iter);
            
            % Result analysis / 结果分析
            fprintf('\nResult analysis:\n');
            
            % Final point error / 最终点误差
            final_error = norm(sol.x{end} - optimizer.xf);
            fprintf('  Final point error: %.6f\n', final_error);
            
            % Control input statistics / 控制输入统计
            u_values = cell2mat(sol.u);
            max_control = max(abs(u_values));
            mean_control = mean(abs(u_values));
            fprintf('  Maximum control input: %.4f rad/s\n', max_control);
            fprintf('  Average control input: %.4f rad/s\n', mean_control);
            
            % Trajectory length / 轨迹长度
            trajectory_length = 0;
            for i = 2:length(sol.t)
                dx = sol.x{i}(1) - sol.x{i-1}(1);
                dy = sol.x{i}(2) - sol.x{i-1}(2);
                trajectory_length = trajectory_length + sqrt(dx^2 + dy^2);
            end
            fprintf('  Trajectory length: %.4f (scaling units)\n', trajectory_length);
            
            % Plot results / 绘制结果
            fprintf('\nPlot results...\n');
            % Do not plot here, plot uniformly in figure 2 / 不在这里绘制，在图2中统一绘制
        else
            % Direct C-PDDP returned solution structure / 直接C-PDDP返回的解结构
            fprintf('  Number of trajectory points: %d\n', length(solution.t));
            fprintf('  Final cost: %.6f\n', solution.J(end));
            fprintf('  Convergence iterations: %d\n', solution.iter);
            
            % Result analysis / 结果分析
            fprintf('\nResult analysis:\n');
            
            % Final point error / 最终点误差
            final_error = norm(solution.x{end} - optimizer.xf);
            fprintf('  Final point error: %.6f\n', final_error);
            
            % Control input statistics / 控制输入统计
            u_values = cell2mat(solution.u);
            max_control = max(abs(u_values));
            mean_control = mean(abs(u_values));
            fprintf('  Maximum control input: %.4f rad/s\n', max_control);
            fprintf('  Average control input: %.4f rad/s\n', mean_control);
            
            % Trajectory length / 轨迹长度
            trajectory_length = 0;
            for i = 2:length(solution.t)
                dx = solution.x{i}(1) - solution.x{i-1}(1);
                dy = solution.x{i}(2) - solution.x{i-1}(2);
                trajectory_length = trajectory_length + sqrt(dx^2 + dy^2);
            end
            fprintf('  Trajectory length: %.4f (scaling units)\n', trajectory_length);
            
            % Plot results / 绘制结果
            fprintf('\nPlot results...\n');
            % Do not plot here, plot uniformly in figure 2 / 不在这里绘制，在图2中统一绘制
        end
        
        
        % Plot results - only show one figure / 绘制结果 - 只显示一个图形
        fprintf('\nPlotting results...\n');
        figure(1);
        set(gcf, 'Name', 'C-PDDP 2D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
        
        % Check solution structure type and plot accordingly / 检查解结构类型并相应绘制
        if isfield(solution, 'sol') && isfield(solution, 'algorithm')
            % DDPACK class returns solution structure containing C-PDDP results / DDPACK类返回包含C-PDDP结果的解结构
            sol = solution.sol;  % Get the actual C-PDDP results / 获取实际的C-PDDP结果
            plot_c_pddp_detailed_analysis(sol, optimizer, x_wpts);
        else
            % Direct C-PDDP returned solution structure / 直接C-PDDP返回的解结构
            plot_c_pddp_detailed_analysis(solution, optimizer, x_wpts);
        end
        
    else
        fprintf('✗ Solving failed\n');
    end
    
catch ME
    fprintf('✗ Example execution failed: %s\n', ME.message);
    fprintf('Error location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    rethrow(ME);
end

%% Custom plotting functions / 自定义绘图函数

function plot_c_pddp_solution(solution, optimizer)
    % Plot C-PDDP solving results - only show trajectory, cost function, energy / 绘制C-PDDP求解结果 - 只显示轨迹、代价函数、能量
    
    figure('Name', 'C-PDDP 2D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(solution.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj = zeros(3, length(solution.x));
        for i = 1:length(solution.x)
            x_traj(:, i) = solution.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj = solution.x;
    end
    
    % Trajectory (Figure 1) / 轨迹（图1）
    subplot(1, 3, 1);
    plot(x_traj(1, :), x_traj(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(optimizer.x0(1), optimizer.x0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(optimizer.xf(1), optimizer.xf(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('X (scaling units)'); ylabel('Y (scaling units)'); title('Trajectory'); grid on;
    
    % Set appropriate axis range to ensure trajectory is visible / 设置适当的坐标轴范围以确保轨迹可见
    x_min = min([x_traj(1, :), optimizer.x0(1), optimizer.xf(1)]);
    x_max = max([x_traj(1, :), optimizer.x0(1), optimizer.xf(1)]);
    y_min = min([x_traj(2, :), optimizer.x0(2), optimizer.xf(2)]);
    y_max = max([x_traj(2, :), optimizer.x0(2), optimizer.xf(2)]);
    
    % Add some margin / 添加一些边距
    x_margin = (x_max - x_min) * 0.1;
    y_margin = (y_max - y_min) * 0.1;
    if x_margin == 0, x_margin = 0.1; end
    if y_margin == 0, y_margin = 0.1; end
    
    xlim([x_min - x_margin, x_max + x_margin]);
    ylim([y_min - y_margin, y_max + y_margin]);
    axis equal;
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
    
    % Cost function (Figure 2) / 代价函数（图2）
    subplot(1, 3, 2);
    plot(solution.J, 'b-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Cost function value'); title('Cost function convergence'); grid on;
    
    % Control energy (Figure 3) / 控制能量（图3）
    subplot(1, 3, 3);
    plot(solution.E, 'r-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Control energy'); title('Control energy convergence'); grid on;
end

function plot_c_pddp_detailed_analysis(solution, optimizer, x_wpts)
    % Plot C-PDDP simplified analysis - only show trajectory, cost function, energy / 绘制C-PDDP简化分析 - 只显示轨迹、代价函数、能量
    
    % Debug information: Check solution structure / 调试信息：检查解结构
    fprintf('Debug information:\n');
    fprintf('  solution.xtype: %s\n', class(solution.x));
    if iscell(solution.x)
        fprintf('  solution.xlength: %d\n', length(solution.x));
        fprintf('  First x element: [%.4f, %.4f]\n', solution.x{1}(1), solution.x{1}(2));
        fprintf('  Last x element: [%.4f, %.4f]\n', solution.x{end}(1), solution.x{end}(2));
    end
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(solution.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj = zeros(3, length(solution.x));
        for i = 1:length(solution.x)
            x_traj(:, i) = solution.x{i};
        end
    else
        % If it's already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj = solution.x;
    end
    
    % Convert control data / 转换控制数据
    if iscell(solution.u)
        u_traj = zeros(1, length(solution.u));
        for i = 1:length(solution.u)
            u_traj(i) = solution.u{i};
        end
    else
        u_traj = solution.u;
    end
    
    fprintf('  Number of trajectory points: %d\n', size(x_traj, 2));
    fprintf('  Trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Theta[%.4f, %.4f]\n', ...
        min(x_traj(1,:)), max(x_traj(1,:)), min(x_traj(2,:)), max(x_traj(2,:)), ...
        min(x_traj(3,:)), max(x_traj(3,:)));
    
    % Trajectory and Waypoints (Figure 1) / 轨迹和航点（图1）
    subplot(1, 3, 1);
    plot(x_traj(1, :), x_traj(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(optimizer.x0(1), optimizer.x0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(optimizer.xf(1), optimizer.xf(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    % Plot intermediate waypoints / 绘制中间航点
    if ~isempty(x_wpts)
        plot(x_wpts(1, :), x_wpts(2, :), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
        legend('Trajectory', 'Start', 'End', 'Waypoints', 'Location', 'best');
    else
        legend('Trajectory', 'Start', 'End', 'Location', 'best');
    end
    xlabel('X (scaling units)'); ylabel('Y (scaling units)'); title('Trajectory'); grid on; 
    
    % Set appropriate axis range to ensure trajectory is visible / 设置适当的坐标轴范围以确保轨迹可见
    x_min = min([x_traj(1, :), optimizer.x0(1), optimizer.xf(1)]);
    x_max = max([x_traj(1, :), optimizer.x0(1), optimizer.xf(1)]);
    y_min = min([x_traj(2, :), optimizer.x0(2), optimizer.xf(2)]);
    y_max = max([x_traj(2, :), optimizer.x0(2), optimizer.xf(2)]);
    
    % Add some margin / 添加一些边距
    x_margin = (x_max - x_min) * 0.1;
    y_margin = (y_max - y_min) * 0.1;
    if x_margin == 0, x_margin = 0.1; end
    if y_margin == 0, y_margin = 0.1; end
    
    xlim([x_min - x_margin, x_max + x_margin]);
    ylim([y_min - y_margin, y_max + y_margin]);
    axis equal;
    
    % Cost function convergence (Figure 2) / 代价函数收敛（图2）
    subplot(1, 3, 2);
    plot(solution.J, 'b-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Cost function value'); title('Cost function convergence'); grid on;
    
    % Control energy convergence (Figure 3) / 控制能量收敛（图3）
    subplot(1, 3, 3);
    plot(solution.E, 'r-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Control energy'); title('Control energy convergence'); grid on;
end

function plot_uavto_detailed_analysis(solution, optimizer, x_wpts)
    % Plot DDPACK class simplified analysis - only show trajectory, cost function, energy / 绘制DDPACK类简化分析 - 只显示轨迹、代价函数、能量
    
    % Trajectory and Waypoints (Figure 1) / 轨迹和航点（图1）
    subplot(1, 3, 1);
    plot(solution.x(1, :), solution.x(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(optimizer.x0(1), optimizer.x0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(optimizer.xf(1), optimizer.xf(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    % Plot intermediate waypoints / 绘制中间航点
    if ~isempty(x_wpts)
        plot(x_wpts(1, :), x_wpts(2, :), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
        legend('Trajectory', 'Start', 'End', 'Waypoints', 'Location', 'best');
    else
        legend('Trajectory', 'Start', 'End', 'Location', 'best');
    end
    xlabel('X (scaling units)'); ylabel('Y (scaling units)'); title('Trajectory'); grid on; axis equal;
    
    % Cost function convergence (Figure 2) / 代价函数收敛（图2）
    subplot(1, 3, 2);
    plot(solution.cost_history, 'b-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Cost function value'); title('Cost function convergence'); grid on;
    
    % Gradient norm convergence (Figure 3) - as energy indicator / 梯度范数收敛（图3）- 作为能量指标
    subplot(1, 3, 3);
    if isfield(solution, 'gradient_norm')
        plot(solution.gradient_norm, 'r-', 'LineWidth', 2);
        xlabel('Number of iterations'); ylabel('Gradient norm'); title('Gradient norm convergence'); grid on;
    else
        % If no gradient norm, show control energy / 如果没有梯度范数，显示控制能量
        plot(solution.u.^2, 'r-', 'LineWidth', 2);
        xlabel('Time step'); ylabel('Control energy'); title('Control energy'); grid on;
    end


fprintf('\nExample completed!\n');
fprintf('Use help DDPACK for more help information.\n');
end
