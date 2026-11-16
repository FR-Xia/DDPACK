%% 3D PDDP Trajectory Optimization Example / 三维PDDP轨迹优化示例
% Demonstrates how to use DDPACK package for 3D Predictive Differential Dynamic Programming / 演示如何使用DDPACK算法包进行三维预测差分动态规划

clear; clc; close all;

fprintf('=== 3D PDDP Trajectory Optimization Example ===\n\n');

try
    % Create optimizer using the same parameters as main_3d_pddp.m / 使用与main_3d_pddp.m相同的参数创建优化器
    fprintf('Creating PDDP optimizer...\n');
    
    % Parameter settings strictly consistent with main_3d_pddp.m / 与main_3d_pddp.m严格一致的参数设置
    s_p = 200;  % Scaling parameter / 缩放参数
    x_0 = [5000.0/s_p; 5000.0/s_p; 5000.0/s_p; 300.0; 0/180*pi; 180/180*pi];  % Initial state / 初始状态
    
    % Target state settings (completely consistent with main_3d_pddp.m) / 目标状态设置（与main_3d_pddp.m完全一致）
    psi_f = 225/180*pi;
    x_star = [0.0; 0.0; 0.0; 300; -60/180*pi; psi_f];  % Target state / 目标状态
    t_f = 35.0;  % Final time / 最终时间
    dt = 0.1;  % Time step / 时间步长
    u_max = [30; 30];  % Maximum control input / 最大控制输入
    m = 150.0;  % Mass / 质量
    g = 9.81;  % Gravity acceleration / 重力加速度
    Q_f = blkdiag(5000 * eye(3), 0, 5000 * eye(2));  % Final state weights / 最终状态权重
    Q_s = blkdiag(0.0001 * eye(3), 0.0, 0.0001 * eye(2));  % Running state weights / 运行状态权重
    R = 0.1 * eye(numel(u_max));  % Control weights / 控制权重
    num_iter = 300;  % Number of iterations / 迭代次数
    alpha = 1.0;  % Line search parameter / 线搜索参数
    
    % Constraints (completely consistent with main_3d_pddp.m) / 约束（与main_3d_pddp.m完全一致）
    constraints = [];
    center = [1500/s_p, 2000/s_p]; r = 500/s_p;
    constraints = [constraints, CircleConstraints_3D(center, r)];
    center = [2800/s_p, 4800/s_p]; r = 500/s_p;
    constraints = [constraints, CircleConstraints_3D(center, r)];
    
    % Create optimizer and set parameters / 创建优化器并设置参数
    optimizer = DDPACK('algorithm', 'pddp', 'dimension', 3);
    optimizer.x0 = x_0;
    optimizer.xf = x_star;
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
    optimizer.alpha = alpha;
    optimizer.constraints = constraints;
    
    % Display parameters / 显示参数
    fprintf('\nParameter settings (consistent with main_3d_pddp.m):\n');
    fprintf('  Scaling parameter: %.0f\n', s_p);
    fprintf('  Initial state: [%.2f, %.2f, %.2f, %.1f, %.2f, %.2f]\n', ...
        optimizer.x0(1), optimizer.x0(2), optimizer.x0(3), optimizer.x0(4), optimizer.x0(5), optimizer.x0(6));
    fprintf('  Target state: [%.2f, %.2f, %.2f, %.1f, %.2f, %.2f]\n', ...
        optimizer.xf(1), optimizer.xf(2), optimizer.xf(3), optimizer.xf(4), optimizer.xf(5), optimizer.xf(6));
    fprintf('  Final time: %.1f s\n', optimizer.tf);
    fprintf('  Time step: %.2f s\n', optimizer.dt);
    fprintf('  Number of iterations: %d\n', optimizer.num_iter);
    fprintf('  Maximum control: [%.1f, %.1f]\n', optimizer.u_max(1), optimizer.u_max(2));
    fprintf('  Mass: %.1f kg\n', optimizer.m);
    fprintf('  Gravity acceleration: %.2f m/s²\n', optimizer.g);
    fprintf('  Line search parameter: %.2f\n', optimizer.alpha);
    fprintf('  Number of constraints: %d\n', length(optimizer.constraints));
    
    % Display cost function weights / 显示代价函数权重
    fprintf('\nCost function weights:\n');
    fprintf('  Q_f (Final state weights):\n');
    disp(optimizer.Q_f);
    fprintf('  Q_s (Running state weights):\n');
    disp(optimizer.Q_s);
    fprintf('  R (Control weights):\n');
    disp(optimizer.R);
    
    % Solve / 求解
    fprintf('\nStarting to solve...\n');
    tic;
    solution = optimizer.solve();
    solve_time = toc;
    
    if ~isempty(solution)
        fprintf('✓ Solved successfully!\n');
        fprintf('  Solve time: %.2f seconds\n', solve_time);
        
        % Check solution structure fields / 检查解结构字段
        if isfield(solution, 'sol') && isfield(solution, 'algorithm')
            % DDPACK class returned solution structure containing PDDP results / DDPACK类返回包含PDDP结果的解结构
            sol = solution.sol;  % Get the actual PDDP results / 获取实际的PDDP结果
            fprintf('  Number of trajectory points: %d\n', length(sol.t));
            fprintf('  Final cost: %.6f\n', sol.J(end));
            fprintf('  Converged iterations: %d\n', sol.iter);
        else
            % Direct PDDP returned solution structure / 直接PDDP返回的解结构
            sol = solution;
            fprintf('  Number of trajectory points: %d\n', length(sol.t));
            fprintf('  Final cost: %.6f\n', sol.J(end));
            fprintf('  Converged iterations: %d\n', sol.iter);
        end
        
        % Analyze results / 分析结果
        fprintf('\nResult analysis:\n');
        
        % Final point error / 最终点误差
        final_error = norm(sol.x{end} - x_star);
        fprintf('  Final point error: %.6f\n', final_error);
        
        % Control input statistics / 控制输入统计
        u_values = cell2mat(sol.u);
        max_control = max(abs(u_values), [], 2);
        mean_control = mean(abs(u_values), 2);
        fprintf('  Maximum control input: [%.4f, %.4f]\n', max_control(1), max_control(2));
        fprintf('  Average control input: [%.4f, %.4f]\n', mean_control(1), mean_control(2));
        
        % Trajectory length / 轨迹长度
        trajectory_length = 0;
        for i = 2:length(sol.t)
            dx = sol.x{i}(1) - sol.x{i-1}(1);
            dy = sol.x{i}(2) - sol.x{i-1}(2);
            dz = sol.x{i}(3) - sol.x{i-1}(3);
            trajectory_length = trajectory_length + sqrt(dx^2 + dy^2 + dz^2);
        end
        fprintf('  Trajectory length: %.4f (scaled units)\n', trajectory_length);
        
        
        % Plot results - only show one figure / 绘制结果 - 只显示一个图形
        fprintf('\nPlotting results...\n');
        figure(1);
        set(gcf, 'Name', 'PDDP 3D Trajectory Optimization Results', 'Position', [100, 100, 1200, 400]);
        
        % Plot PDDP results / 绘制PDDP结果
        plot_pddp_detailed_analysis(sol, optimizer, x_0, x_star, s_p);
        
    else
        fprintf('✗ Solve failed\n');
    end
    
catch ME
    fprintf('✗ Example run failed: %s\n', ME.message);
    fprintf('Error location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    rethrow(ME);
end

fprintf('\nExample completed!\n');
fprintf('Use help DDPACK to view more help information.\n');

%% Custom plotting function / 自定义绘图函数

function plot_pddp_detailed_analysis(solution, optimizer, x_0, x_star, s_p)
    % Plot PDDP simplified analysis - only show trajectory, cost function, energy / 绘制PDDP简化分析 - 只显示轨迹、代价函数、能量
    
    % Debug information: check solution structure / 调试信息：检查解结构
    fprintf('Debug information:\n');
    fprintf('  solution.x type: %s\n', class(solution.x));
    if iscell(solution.x)
        fprintf('  solution.x length: %d\n', length(solution.x));
        fprintf('  First x element: [%.4f, %.4f, %.4f]\n', solution.x{1}(1), solution.x{1}(2), solution.x{1}(3));
        fprintf('  Last x element: [%.4f, %.4f, %.4f]\n', solution.x{end}(1), solution.x{end}(2), solution.x{end}(3));
    end
    
    % Convert trajectory data / 转换轨迹数据
    if iscell(solution.x)
        % If it is a cell array, convert to matrix / 如果是单元数组，转换为矩阵
        x_traj = zeros(6, length(solution.x));
        for i = 1:length(solution.x)
            x_traj(:, i) = solution.x{i};
        end
    else
        % If it is already a matrix, use directly / 如果已经是矩阵，直接使用
        x_traj = solution.x;
    end
    
    % Convert control data / 转换控制数据
    if iscell(solution.u)
        u_traj = zeros(2, length(solution.u));
        for i = 1:length(solution.u)
            u_traj(:, i) = solution.u{i};
        end
    else
        u_traj = solution.u;
    end
    
    fprintf('  Number of trajectory points: %d\n', size(x_traj, 2));
    fprintf('  Trajectory range: X[%.4f, %.4f], Y[%.4f, %.4f], Z[%.4f, %.4f]\n', ...
        min(x_traj(1,:)), max(x_traj(1,:)), min(x_traj(2,:)), max(x_traj(2,:)), ...
        min(x_traj(3,:)), max(x_traj(3,:)));
    
    % 3D trajectory (Figure 1) / 三维轨迹（图1）
    subplot(1, 3, 1);
    hold on;
    
    % Plot obstacles / 绘制障碍物
    h_obstacles = [];
    for i = 1:length(optimizer.constraints)
        [hCylinder, hEndPlate1, hEndPlate2] = cylinder3([optimizer.constraints(i).Center(1) optimizer.constraints(i).Center(2) x_0(3)]*s_p, ...
            [optimizer.constraints(i).Center(1) optimizer.constraints(i).Center(2) x_star(3)]*s_p, optimizer.constraints(i).r*s_p, 1000, 'r', 1, 0);
        h_obstacles = [h_obstacles, hCylinder];
    end
    
    % Plot trajectory / 绘制轨迹
    h_traj = plot3(x_traj(1, :)*s_p, x_traj(2, :)*s_p, x_traj(3, :)*s_p, 'b-', 'LineWidth', 2);
    h_start = plot3(x_0(1)*s_p, x_0(2)*s_p, x_0(3)*s_p, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h_end = plot3(x_star(1)*s_p, x_star(2)*s_p, x_star(3)*s_p, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('3D Trajectory'); grid on;
    
    % Create correct legend / 创建正确的图例
    if ~isempty(h_obstacles)
        legend([h_obstacles(1), h_traj, h_start, h_end], 'Obstacles', 'Trajectory', 'Start', 'End', 'Location', 'best');
    else
        legend([h_traj, h_start, h_end], 'Trajectory', 'Start', 'End', 'Location', 'best');
    end
    
    % Set appropriate axis ranges / 设置适当的坐标轴范围
    x_min = min([x_traj(1, :), x_0(1), x_star(1)]) * s_p;
    x_max = max([x_traj(1, :), x_0(1), x_star(1)]) * s_p;
    y_min = min([x_traj(2, :), x_0(2), x_star(2)]) * s_p;
    y_max = max([x_traj(2, :), x_0(2), x_star(2)]) * s_p;
    z_min = min([x_traj(3, :), x_0(3), x_star(3)]) * s_p;
    z_max = max([x_traj(3, :), x_0(3), x_star(3)]) * s_p;
    
    x_margin = (x_max - x_min) * 0.1;
    y_margin = (y_max - y_min) * 0.1;
    z_margin = (z_max - z_min) * 0.1;
    if x_margin == 0, x_margin = 100; end
    if y_margin == 0, y_margin = 100; end
    if z_margin == 0, z_margin = 100; end
    
    xlim([x_min - x_margin, x_max + x_margin]);
    ylim([y_min - y_margin, y_max + y_margin]);
    zlim([z_min - z_margin, z_max + z_margin]);
    
    % Set 3D view angle for better perspective / 设置3D视角以获得更好的透视效果
    view(45, 30);  % Azimuth 45°, Elevation 30° / 方位角45°，仰角30°
    
    % Cost function convergence (Figure 2) / 代价函数收敛（图2）
    subplot(1, 3, 2);
    plot(solution.J, 'b-', 'LineWidth', 2);
    xlabel('Iteration number'); ylabel('Cost function value'); title('Cost Function Convergence'); grid on;
    
    % Control energy convergence (Figure 3) / 控制能量收敛（图3）
    subplot(1, 3, 3);
    plot(solution.E, 'r-', 'LineWidth', 2);
    xlabel('Iteration number'); ylabel('Control energy'); title('Control Energy Convergence'); grid on;
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
