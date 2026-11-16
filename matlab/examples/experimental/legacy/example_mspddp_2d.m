%% 2D MSPDDP Trajectory Optimization Example / 二维MSPDDP轨迹优化示例
% Demonstrates how to use UAVTO toolbox for 2D Model Predictive Differential Dynamic Programming / 演示如何使用UAVTO工具箱进行二维模型预测差分动态规划

clear; clc; close all;

fprintf('=== 2D MSPDDP Trajectory Optimization Example ===\n\n');

try
    % Create optimizer using the same parameters as main_2d_mspddp.m / 使用与main_2d_mspddp.m相同的参数创建优化器
    fprintf('Creating MSPDDP optimizer...\n');
    
    % Parameter settings strictly consistent with main_2d_mspddp.m / 与main_2d_mspddp.m严格一致的参数设置
    x_0 = [0; 0; 0];  % Initial state / 初始状态
    x_star = [8; 12; pi/3];  % Target state 1 / 目标状态1
    x_star2 = [23; 18; pi/6];  % Target state 2 / 目标状态2
    x_star3 = [28; 33; pi/2];  % Target state 3 / 目标状态3
    t_f = 18;  % Final time / 最终时间
    dt = 0.1;  % Time step / 时间步长
    u_max = pi/6;  % Maximum control input / 最大控制输入
    v = 1.0;  % Velocity / 速度
    Q_f = [10.0   0.0     0.0;
           0.0    10.0    0.0;
           0.0    0.0     10.0];  % Final state weight / 最终状态权重
    Q_s = [0.0    0.0     0.0;
           0.0    0.0     0.0;
           0.0    0.0     0.0];  % Running state weight / 运行状态权重
    R = 1.0;  % Control weight / 控制权重
    num_iter = 100;  % Number of iterations / 迭代次数
    epsilon = 0.9;  % Line search parameter / 线搜索参数
    rho = 0.5;  % Regularization parameter / 正则化参数
    update_epsilon = 0.9;  % Update parameter / 更新参数
    
    % Create optimizer and set parameters / 创建优化器并设置参数
    optimizer = UAVTO('algorithm', 'mspddp');
    optimizer.x0 = x_0;
    optimizer.xf = x_star;
    optimizer.tf = t_f;
    optimizer.dt = dt;
    optimizer.u_max = u_max;
    optimizer.v = v;
    optimizer.Q_f = Q_f;
    optimizer.Q_s = Q_s;
    optimizer.R = R;
    optimizer.num_iter = num_iter;
    optimizer.epsilon = epsilon;
    optimizer.rho = rho;
    optimizer.update_epsilon = update_epsilon;
    
    % Display parameters / 显示参数
    fprintf('\nParameter settings (consistent with main_2d_mspddp.m):\n');
    fprintf('  Initial state: [%.2f, %.2f, %.2f]\n', optimizer.x0(1), optimizer.x0(2), optimizer.x0(3));
    fprintf('  Target state 1: [%.2f, %.2f, %.2f]\n', x_star(1), x_star(2), x_star(3));
    fprintf('  Target state 2: [%.2f, %.2f, %.2f]\n', x_star2(1), x_star2(2), x_star2(3));
    fprintf('  Target state 3: [%.2f, %.2f, %.2f]\n', x_star3(1), x_star3(2), x_star3(3));
    fprintf('  Final time: %.1f s\n', optimizer.tf);
    fprintf('  Time step: %.2f s\n', optimizer.dt);
    fprintf('  Number of iterations: %d\n', optimizer.num_iter);
    fprintf('  Maximum control: %.4f rad/s\n', optimizer.u_max);
    fprintf('  Velocity: %.1f m/s\n', optimizer.v);
    fprintf('  Line search parameter: %.2f\n', optimizer.epsilon);
    fprintf('  Regularization parameter: %.2f\n', optimizer.rho);
    
    % Display cost function weights / 显示代价函数权重
    fprintf('\nCost function weights:\n');
    fprintf('  Q_f (Final state weight):\n');
    disp(optimizer.Q_f);
    fprintf('  Q_s (Running state weight):\n');
    disp(optimizer.Q_s);
    fprintf('  R (Control weight): %.2f\n', optimizer.R);
    
    % Solve three trajectory segments / 求解三段轨迹
    fprintf('\nStarting to solve three trajectory segments...\n');
    
    % First segment / 第一段
    fprintf('Solving first trajectory segment...\n');
    tic;
    solution1 = optimizer.solve();
    solve_time1 = toc;
    
    if ~isempty(solution1)
        fprintf('✓ First segment solved successfully!\n');
        fprintf('  Solve time: %.2f seconds\n', solve_time1);
        
        % Check solution structure fields / 检查解结构字段
        if isfield(solution1, 'sol') && isfield(solution1, 'algorithm')
            sol1 = solution1.sol;
        else
            sol1 = solution1;
        end
        
        % Second segment / 第二段
        optimizer.x0 = x_star;
        optimizer.xf = x_star2;
        fprintf('Solving second trajectory segment...\n');
        tic;
        solution2 = optimizer.solve();
        solve_time2 = toc;
        
        if ~isempty(solution2)
            fprintf('✓ Second segment solved successfully!\n');
            fprintf('  Solve time: %.2f seconds\n', solve_time2);
            
            % Check solution structure fields / 检查解结构字段
            if isfield(solution2, 'sol') && isfield(solution2, 'algorithm')
                sol2 = solution2.sol;
            else
                sol2 = solution2;
            end
            
            % Third segment / 第三段
            optimizer.x0 = x_star2;
            optimizer.xf = x_star3;
            fprintf('Solving third trajectory segment...\n');
            tic;
            solution3 = optimizer.solve();
            solve_time3 = toc;
            
            if ~isempty(solution3)
                fprintf('✓ Third segment solved successfully!\n');
                fprintf('  Solve time: %.2f seconds\n', solve_time3);
                
                % Check solution structure fields / 检查解结构字段
                if isfield(solution3, 'sol') && isfield(solution3, 'algorithm')
                    sol3 = solution3.sol;
                else
                    sol3 = solution3;
                end
                
                
                % Plot results - only show one figure1 / 绘制结果 - 只显示图1
                fprintf('\nPlotting results...\n');
                figure(1);
                set(gcf, 'Name', 'MSPDDP 2D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
                
                % Plot MSPDDP results / 绘制MSPDDP结果
                plot_mspddp_detailed_analysis(sol1, sol2, sol3, optimizer, x_0, x_star, x_star2, x_star3);
                
            else
                fprintf('✗ Third segment solving failed\n');
            end
            
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

function plot_mspddp_detailed_analysis(sol1, sol2, sol3, optimizer, x_0, x_star, x_star2, x_star3)
    % Plot MSPDDP simplified analysis - only show trajectory, cost function, energy / 绘制MSPDDP简化分析 - 只显示轨迹、代价函数、能量
    
    % Debug information: Check solution structure / 调试信息：检查解结构
    fprintf('Debug information:\n');
    fprintf('  sol1.x type: %s\n', class(sol1.x));
    fprintf('  sol2.x type: %s\n', class(sol2.x));
    fprintf('  sol3.x type: %s\n', class(sol3.x));
    if iscell(sol1.x)
        fprintf('  sol1.x length: %d\n', length(sol1.x));
        fprintf('  sol2.x length: %d\n', length(sol2.x));
        fprintf('  sol3.x length: %d\n', length(sol3.x));
    end
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(sol1.x)
        % If it's a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj1 = zeros(3, length(sol1.x));
        for i = 1:length(sol1.x)
            x_traj1(:, i) = sol1.x{i};
        end
    else
        % If already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj1 = sol1.x;
    end
    
    if iscell(sol2.x)
        x_traj2 = zeros(3, length(sol2.x));
        for i = 1:length(sol2.x)
            x_traj2(:, i) = sol2.x{i};
        end
    else
        x_traj2 = sol2.x;
    end
    
    if iscell(sol3.x)
        x_traj3 = zeros(3, length(sol3.x));
        for i = 1:length(sol3.x)
            x_traj3(:, i) = sol3.x{i};
        end
    else
        x_traj3 = sol3.x;
    end
    
    fprintf('  First segment trajectory points: %d\n', size(x_traj1, 2));
    fprintf('  Second segment trajectory points: %d\n', size(x_traj2, 2));
    fprintf('  Third segment trajectory points: %d\n', size(x_traj3, 2));
    
    % Trajectory and waypoints (Figure 1) / 轨迹和航点（图1）
    subplot(1, 3, 1);
    hold on;
    
    % Plot three trajectory segments / 绘制三段轨迹
    plot(x_traj1(1, :), x_traj1(2, :), 'b-', 'LineWidth', 2);
    plot(x_traj2(1, :), x_traj2(2, :), 'r-', 'LineWidth', 2);
    plot(x_traj3(1, :), x_traj3(2, :), 'g-', 'LineWidth', 2);
    
    % Plot waypoints - consistent with c_pddp style / 绘制航点 - 与c_pddp风格一致
    h_start = plot(x_0(1), x_0(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h_waypoint1 = plot(x_star(1), x_star(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
    h_waypoint2 = plot(x_star2(1), x_star2(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
    h_end = plot(x_star3(1), x_star3(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); title('Trajectory'); grid on; axis equal;
    legend('First segment', 'Second segment', 'Third segment', 'Start', 'Waypoint 1', 'Waypoint 2', 'End', 'Location', 'best');
    
    % Set appropriate axis range / 设置适当的坐标轴范围
    all_x = [x_traj1(1, :), x_traj2(1, :), x_traj3(1, :), x_0(1), x_star(1), x_star2(1), x_star3(1)];
    all_y = [x_traj1(2, :), x_traj2(2, :), x_traj3(2, :), x_0(2), x_star(2), x_star2(2), x_star3(2)];
    
    x_min = min(all_x); x_max = max(all_x);
    y_min = min(all_y); y_max = max(all_y);
    
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
    plot(sol3.J, 'g-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Cost function value'); title('Cost function convergence'); grid on;
    legend('First segment', 'Second segment', 'Third segment', 'Location', 'best');
    
    % Control energy convergence (Figure 3) / 控制能量收敛（图3）
    subplot(1, 3, 3);
    plot(sol1.E, 'b-', 'LineWidth', 2);
    hold on;
    plot(sol2.E, 'r-', 'LineWidth', 2);
    plot(sol3.E, 'g-', 'LineWidth', 2);
    xlabel('Number of iterations'); ylabel('Control energy'); title('Control energy convergence'); grid on;
    legend('First segment', 'Second segment', 'Third segment', 'Location', 'best');
end
