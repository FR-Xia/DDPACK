function algorithm_comparison()
    % Algorithm performance comparison function / 算法性能比较函数
    % Compare performance, error, computation time, optimization results of different algorithms / 比较不同算法的性能、误差、计算时间、优化结果
    
    fprintf('=== Algorithm Performance Comparison ===\n\n');
    
    % Group 1 comparison: PDDP vs FFT-CDDP (3D) / 第一组比较：PDDP vs FFT-CDDP (3D)
    fprintf('Group 1 comparison: PDDP vs FFT-CDDP (3D)\n');
    fprintf('Using waypoint positions and obstacle positions from example_fft_cddp_3d\n\n');
    
    results_group1 = compare_3d_algorithms();
    
    % Group 2 comparison: MSPDDP vs C-PDDP (2D) / 第二组比较：MSPDDP vs C-PDDP (2D)
    fprintf('\nGroup 2 comparison: MSPDDP vs C-PDDP (2D)\n');
    fprintf('Using waypoint positions from example_mspddp_2d\n\n');
    
    results_group2 = compare_2d_algorithms();
    
    % Display comparison results / 显示比较结果
    display_comparison_results(results_group1, results_group2);
    
    % Plot trajectory comparison / 绘制轨迹比较图
    plot_trajectory_comparison(results_group1, results_group2);
end

function results = compare_3d_algorithms()
    % Compare 3D algorithms: PDDP vs FFT-CDDP / 比较3D算法：PDDP vs FFT-CDDP
    % Using parameter settings from example_fft_cddp_3d / 使用example_fft_cddp_3d的参数设置
    
    global psi_f
    psi_f = 225/180*pi;
    
    % Parameter settings (PDDP uses example_pddp_3d parameters, FFT-CDDP uses example_fft_cddp_3d parameters) / 参数设置（PDDP使用example_pddp_3d参数，FFT-CDDP使用example_fft_cddp_3d参数）
    s_p = 200;
    x_0 = [5000.0/s_p; 5000.0/s_p; 5000.0/s_p; 300.0; 0/180*pi; 180/180*pi];
    x_star = [0.0; 0.0; 0.0; 300; -60/180*pi; psi_f];
    t_f = 35.0;
    u_max = [30; 30];
    m = 150.0;
    g = 9.81;  % PDDP uses 9.81 / PDDP使用9.81
    R = 0.1 * eye(numel(u_max));
    num_iter = 300;
    
    % PDDP parameters / PDDP参数
    Q_f_pddp = blkdiag(5000 * eye(3), 0, 5000 * eye(2));  % PDDP weights / PDDP权重
    Q_s_pddp = blkdiag(0.0001 * eye(3), 0.0, 0.0001 * eye(2));
    alpha_pddp = 1.0;  % PDDP line search parameter / PDDP线搜索参数
    
    % FFT-CDDP parameters / FFT-CDDP参数
    Q_f_fft = blkdiag(2.0*s_p*eye(3), 0.0, 5*10^4*eye(2));  % FFT-CDDP weights / FFT-CDDP权重
    Q_s_fft = blkdiag(0.0000001*s_p*eye(3), 0.0, 0.0000001*eye(2));
    alpha_fft = 0.5;  % FFT-CDDP line search parameter / FFT-CDDP线搜索参数
    
    % Obstacle constraints / 障碍物约束
    constraints = [];
    center = [1500/s_p, 2000/s_p]; r = 500/s_p;
    constraints = [constraints, CircleConstraints_3D(center, r)];
    center = [2800/s_p, 4800/s_p]; r = 500/s_p;
    constraints = [constraints, CircleConstraints_3D(center, r)];
    
    % Test PDDP algorithm / 测试PDDP算法
    fprintf('Testing PDDP algorithm...\n');
    pddp_results = test_algorithm('pddp', 3, x_0, x_star, t_f, 0.1, u_max, m, g, s_p, Q_f_pddp, Q_s_pddp, R, num_iter, alpha_pddp, constraints);
    
    % Test FFT-CDDP algorithm / 测试FFT-CDDP算法
    fprintf('Testing FFT-CDDP algorithm...\n');
    fft_cddp_results = test_algorithm('fft_cddp', 3, x_0, x_star, t_f, 0.2, u_max, m, g, s_p, Q_f_fft, Q_s_fft, R, num_iter, alpha_fft, constraints);
    
    results = struct();
    results.pddp = pddp_results;
    results.fft_cddp = fft_cddp_results;
    results.algorithm_names = {'PDDP', 'FFT-CDDP'};
    results.dimension = 3;
end

function results = compare_2d_algorithms()
    % Compare 2D algorithms: MSPDDP vs C-PDDP / 比较2D算法：MSPDDP vs C-PDDP
    % MSPDDP uses waypoint positions and parameters from example_mspddp_2d / MSPDDP使用example_mspddp_2d的航迹点位置和参数
    % C-PDDP uses parameters from example_c_pddp_2d, but waypoint positions match MSPDDP / C-PDDP使用example_c_pddp_2d的参数，但航迹点位置与MSPDDP一致
    
    % MSPDDP parameter settings (consistent with example_mspddp_2d) / MSPDDP参数设置（与example_mspddp_2d一致）
    x_0_mspddp = [0; 0; 0];
    x_star1 = [8; 12; pi/3];
    x_star2 = [23; 18; pi/6];
    x_star3 = [28; 33; pi/2];
    t_f_mspddp = 18;
    dt_mspddp = 0.1;
    u_max_mspddp = pi/6;
    v_mspddp = 1.0;
    Q_f_mspddp = [10.0   0.0     0.0;
                  0.0    10.0    0.0;
                  0.0    0.0     10.0];
    Q_s_mspddp = [0.0    0.0     0.0;
                  0.0    0.0     0.0;
                  0.0    0.0     0.0];
    R_mspddp = 1.0;
    num_iter_mspddp = 100;
    epsilon = 0.9;
    rho = 0.5;
    update_epsilon = 0.9;
    
    % C-PDDP parameter settings (consistent with example_c_pddp_2d, but waypoint positions as specified by user) / C-PDDP参数设置（与example_c_pddp_2d一致，但航迹点位置按用户指定）
    s_p = 30;  % C-PDDP scaling parameter / C-PDDP缩放参数
    x_0_cpddp = [0.0; 0.0; 0.0];
    % User-specified waypoint coordinates (need scaling) / 用户指定的航迹点坐标（需要缩放）
    x_wpts = [[80/s_p; 120/s_p], [230/s_p; 180/s_p]];  % Intermediate waypoints (scaled) / 中间航点（缩放后）
    x_star_cpddp = [280/s_p; 330/s_p; pi/2];  % Final target (scaled) / 最终目标（缩放后）
    t_f_cpddp = 27.0;  % C-PDDP final time / C-PDDP最终时间
    dt_cpddp = 0.1;
    u_max_cpddp = 30/180*pi;  % C-PDDP control input / C-PDDP控制输入
    v_cpddp = 30.0;  % C-PDDP speed / C-PDDP速度
    Q_f_cpddp = [20.0    0.0     0.0;
                 0.0      20.0   0.0;
                 0.0      0.0     20.0];
    Q_s_cpddp = [0.0    0.0     0.0;
                 0.0    0.0     0.0;
                 0.0    0.0     0.0];
    R_cpddp = 1.0;
    num_iter_cpddp = 500;
    
    % Test MSPDDP algorithm (three segments) / 测试MSPDDP算法（三段轨迹）
    fprintf('Testing MSPDDP algorithm...\n');
    mspddp_results = test_mspddp_algorithm(x_0_mspddp, x_star1, x_star2, x_star3, t_f_mspddp, dt_mspddp, u_max_mspddp, v_mspddp, Q_f_mspddp, Q_s_mspddp, R_mspddp, num_iter_mspddp, epsilon, rho, update_epsilon);
    
    % Test C-PDDP algorithm (using C-PDDP parameters, but waypoint positions match MSPDDP) / 测试C-PDDP算法（使用C-PDDP参数，但航迹点位置与MSPDDP一致）
    fprintf('Testing C-PDDP algorithm...\n');
    c_pddp_results = test_c_pddp_algorithm(x_0_cpddp, x_wpts, x_star_cpddp, t_f_cpddp, dt_cpddp, u_max_cpddp, v_cpddp, Q_f_cpddp, Q_s_cpddp, R_cpddp, num_iter_cpddp, s_p);
    
    results = struct();
    results.mspddp = mspddp_results;
    results.c_pddp = c_pddp_results;
    results.algorithm_names = {'MSPDDP', 'C-PDDP'};
    results.dimension = 2;
end

function result = test_algorithm(algorithm, dimension, x_0, x_star, t_f, dt, u_max, m, g, s_p, Q_f, Q_s, R, num_iter, alpha, constraints)
    % Test single algorithm and return results / 测试单个算法并返回结果
    
    result = struct();
    result.algorithm = algorithm;
    result.success = false;
    result.solve_time = 0;
    result.final_cost = inf;
    result.final_error = inf;
    result.trajectory_length = 0;
    result.convergence_iterations = 0;
    result.solution = [];
    
    try
        % Create optimizer / 创建优化器
        optimizer = DDPACK('algorithm', algorithm, 'dimension', dimension);
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
        
        % Solve / 求解
        tic;
        solution = optimizer.solve();
        solve_time = toc;
        
        if ~isempty(solution)
            result.success = true;
            result.solve_time = solve_time;
            
            % Get solution structure / 获取解结构
            if isfield(solution, 'sol') && isfield(solution, 'algorithm')
                sol = solution.sol;
            else
                sol = solution;
            end
            
            result.solution = sol;
            result.final_cost = calculate_final_cost(sol.J);
            
            % Calculate final point error / 计算最终点误差
            if iscell(sol.x)
                final_state = sol.x{end};
            else
                final_state = sol.x(:, end);
            end
            result.final_error = norm(final_state - x_star);
            
            % Calculate trajectory length / 计算轨迹长度
            if iscell(sol.x)
                x_traj = zeros(length(x_0), length(sol.x));
                for i = 1:length(sol.x)
                    x_traj(:, i) = sol.x{i};
                end
            else
                x_traj = sol.x;
            end
            
            trajectory_length = 0;
            for i = 2:size(x_traj, 2)
                dx = x_traj(1, i) - x_traj(1, i-1);
                dy = x_traj(2, i) - x_traj(2, i-1);
                if dimension == 3
                    dz = x_traj(3, i) - x_traj(3, i-1);
                    trajectory_length = trajectory_length + sqrt(dx^2 + dy^2 + dz^2);
                else
                    trajectory_length = trajectory_length + sqrt(dx^2 + dy^2);
                end
            end
            result.trajectory_length = trajectory_length;
            
            
            % Convergence iterations / 收敛迭代次数
            result.convergence_iterations = find_convergence_iterations(sol.J);
        end
        
    catch ME
        fprintf('Algorithm %s execution failed: %s\n', algorithm, ME.message);
        result.error_message = ME.message;
    end
end

function result = test_mspddp_algorithm(x_0, x_star1, x_star2, x_star3, t_f, dt, u_max, v, Q_f, Q_s, R, num_iter, epsilon, rho, update_epsilon)
    % Test MSPDDP algorithm (three segments) / 测试MSPDDP算法（三段轨迹）
    
    result = struct();
    result.algorithm = 'mspddp';
    result.success = false;
    result.solve_time = 0;
    result.final_cost = inf;
    result.final_error = inf;
    result.trajectory_length = 0;
    result.convergence_iterations = 0;
    result.solution = [];
    
    try
        % Create optimizer / 创建优化器
        optimizer = DDPACK('algorithm', 'mspddp');
        optimizer.x0 = x_0;
        optimizer.xf = x_star1;
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
        
        % Solve three segments / 求解三段轨迹
        tic;
        
        % First segment / 第一段
        solution1 = optimizer.solve();
        if isempty(solution1)
            return;
        end
        
        % Second segment / 第二段
        optimizer.x0 = x_star1;
        optimizer.xf = x_star2;
        solution2 = optimizer.solve();
        if isempty(solution2)
            return;
        end
        
        % Third segment / 第三段
        optimizer.x0 = x_star2;
        optimizer.xf = x_star3;
        solution3 = optimizer.solve();
        if isempty(solution3)
            return;
        end
        
        solve_time = toc;
        
        % Get solution structure / 获取解结构
        if isfield(solution1, 'sol')
            sol1 = solution1.sol;
        else
            sol1 = solution1;
        end
        if isfield(solution2, 'sol')
            sol2 = solution2.sol;
        else
            sol2 = solution2;
        end
        if isfield(solution3, 'sol')
            sol3 = solution3.sol;
        else
            sol3 = solution3;
        end
        
        result.success = true;
        result.solve_time = solve_time;
        result.solution = {sol1, sol2, sol3};
        
        % Calculate total cost / 计算总代价
        result.final_cost = calculate_final_cost(sol1.J) + calculate_final_cost(sol2.J) + calculate_final_cost(sol3.J);
        
        % Calculate final point error / 计算最终点误差
        if iscell(sol3.x)
            final_state = sol3.x{end};
        else
            final_state = sol3.x(:, end);
        end
        result.final_error = norm(final_state - x_star3);
        
        % Calculate total trajectory length / 计算总轨迹长度
        total_length = 0;
        for sol = {sol1, sol2, sol3}
            if iscell(sol{1}.x)
                x_traj = zeros(3, length(sol{1}.x));
                for i = 1:length(sol{1}.x)
                    x_traj(:, i) = sol{1}.x{i};
                end
            else
                x_traj = sol{1}.x;
            end
            
            for i = 2:size(x_traj, 2)
                dx = x_traj(1, i) - x_traj(1, i-1);
                dy = x_traj(2, i) - x_traj(2, i-1);
                total_length = total_length + sqrt(dx^2 + dy^2);
            end
        end
        result.trajectory_length = total_length;
        
        
        % Convergence iterations (maximum of three segments) / 收敛迭代次数（三段中最大的）
        % Calculate convergence iterations for each segment (iteration count when cost function first becomes 0) / 计算每段的收敛次数（代价函数第一次变为0时的迭代次数）
        conv1 = find_convergence_iterations(sol1.J);
        conv2 = find_convergence_iterations(sol2.J);
        conv3 = find_convergence_iterations(sol3.J);
        result.convergence_iterations = max([conv1, conv2, conv3]);
        
    catch ME
        fprintf('MSPDDP algorithm execution failed: %s\n', ME.message);
        result.error_message = ME.message;
    end
end

function result = test_c_pddp_algorithm(x_0, x_wpts, x_star, t_f, dt, u_max, v, Q_f, Q_s, R, num_iter, s_p)
    % Test C-PDDP algorithm / 测试C-PDDP算法
    
    result = struct();
    result.algorithm = 'c_pddp';
    result.success = false;
    result.solve_time = 0;
    result.final_cost = inf;
    result.final_error = inf;
    result.trajectory_length = 0;
    result.convergence_iterations = 0;
    result.solution = [];
    
    try
        % Create optimizer / 创建优化器
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
        optimizer.s_p = s_p;
        
        % Display C-PDDP parameter settings / 显示C-PDDP参数设置
        fprintf('  C-PDDP parameter settings:\n');
        fprintf('    Scaling parameter s_p: %.0f\n', s_p);
        fprintf('    Initial state: [%.2f, %.2f, %.2f]\n', x_0(1), x_0(2), x_0(3));
        fprintf('    Final target: [%.2f, %.2f, %.2f] (original coordinates: [%.0f, %.0f])\n', x_star(1), x_star(2), x_star(3), x_star(1)*s_p, x_star(2)*s_p);
        fprintf('    Intermediate waypoints: [%.2f, %.2f], [%.2f, %.2f] (original coordinates: [%.0f, %.0f], [%.0f, %.0f])\n', ...
            x_wpts(1,1), x_wpts(2,1), x_wpts(1,2), x_wpts(2,2), ...
            x_wpts(1,1)*s_p, x_wpts(2,1)*s_p, x_wpts(1,2)*s_p, x_wpts(2,2)*s_p);
        fprintf('    Final time: %.1f s\n', t_f);
        fprintf('    Iterations: %d\n', num_iter);
        
        % Solve / 求解
        tic;
        solution = optimizer.solve();
        solve_time = toc;
        
        if ~isempty(solution)
            result.success = true;
            result.solve_time = solve_time;
            
            % Get solution structure / 获取解结构
            if isfield(solution, 'sol')
                sol = solution.sol;
            else
                sol = solution;
            end
            
            result.solution = sol;
            result.final_cost = calculate_final_cost(sol.J);
            
            % Calculate final point error / 计算最终点误差
            if iscell(sol.x)
                final_state = sol.x{end};
            else
                final_state = sol.x(:, end);
            end
            result.final_error = norm(final_state - x_star);
            
            % Calculate trajectory length / 计算轨迹长度
            if iscell(sol.x)
                x_traj = zeros(3, length(sol.x));
                for i = 1:length(sol.x)
                    x_traj(:, i) = sol.x{i};
                end
            else
                x_traj = sol.x;
            end
            
            trajectory_length = 0;
            for i = 2:size(x_traj, 2)
                dx = x_traj(1, i) - x_traj(1, i-1);
                dy = x_traj(2, i) - x_traj(2, i-1);
                trajectory_length = trajectory_length + sqrt(dx^2 + dy^2);
            end
            result.trajectory_length = trajectory_length;
            
            
            % Convergence iterations / 收敛迭代次数
            result.convergence_iterations = find_convergence_iterations(sol.J);
        end
        
    catch ME
        fprintf('C-PDDP algorithm execution failed: %s\n', ME.message);
        result.error_message = ME.message;
    end
end

function final_cost = calculate_final_cost(J)
    % Calculate final cost: take the value before the first time it becomes 0 / 计算最终代价：取第一次变为0时的前一时刻的值
    % If there is no 0 in J sequence, return the last non-zero value / 如果J序列中没有0，返回最后一个非零值
    % If J sequence is all zeros, return 0 / 如果J序列全为0，返回0
    
    % Find the position of the first 0 / 找到第一个0的位置
    zero_indices = find(J == 0);
    
    if isempty(zero_indices)
        % If there is no 0, return the last value / 如果没有0，返回最后一个值
        final_cost = J(end);
    elseif zero_indices(1) == 1
        % If the first value is 0, return 0 / 如果第一个值就是0，返回0
        final_cost = 0;
    else
        % Return the value before the first 0 / 返回第一个0的前一个值
        final_cost = J(zero_indices(1) - 1);
    end
end

function conv_iter = find_convergence_iterations(J)
    % Calculate convergence iterations (iteration count when cost function first becomes 0) / 计算收敛迭代次数（代价函数第一次变为0时的迭代次数）
    zero_indices = find(J == 0);
    
    if isempty(zero_indices)
        % If no 0 is found, return total iterations / 如果没有找到0，返回总迭代次数
        conv_iter = length(J);
    else
        % Return the index of the first 0 (i.e., iteration count at convergence) / 返回第一个0的索引（即收敛时的迭代次数）
        conv_iter = zero_indices(1);
    end
end

function conv_time = find_theta_convergence_time(Theta)
    % Calculate the non-zero value before Theta becomes 0 (flight time at convergence) / 计算Theta变为0时的上一个非0值（收敛时的飞行时间）
    zero_indices = find(Theta == 0);
    
    if isempty(zero_indices)
        % If no 0 is found, return the last value / 如果没有找到0，返回最后一个值
        conv_time = Theta(end);
    elseif zero_indices(1) == 1
        % If the first value is 0, return 0 / 如果第一个值就是0，返回0
        conv_time = 0;
    else
        % Return the value before the first 0 / 返回第一个0的前一个值
        conv_time = Theta(zero_indices(1) - 1);
    end
end

function display_comparison_results(results_group1, results_group2)
    % Display comparison results / 显示比较结果
    
    fprintf('\n=== Detailed Comparison Results ===\n\n');
    
    % Group 1 results - 3D algorithms (side by side) / 第一组结果 - 3D算法（左右排布）
    fprintf('Group 1: 3D Algorithm Comparison (PDDP vs FFT-CDDP)\n');
    fprintf('====================================================\n\n');
    
    if results_group1.pddp.success && results_group1.fft_cddp.success
        display_algorithm_comparison_side_by_side('PDDP', results_group1.pddp, 'FFT-CDDP', results_group1.fft_cddp, 3);
    else
        % If one fails, display separately / 如果其中一个失败，分别显示
        if results_group1.pddp.success
            display_algorithm_detailed_results('PDDP', results_group1.pddp, 3);
        else
            fprintf('PDDP algorithm execution failed\n\n');
        end
        
        if results_group1.fft_cddp.success
            display_algorithm_detailed_results('FFT-CDDP', results_group1.fft_cddp, 3);
        else
            fprintf('FFT-CDDP algorithm execution failed\n\n');
        end
    end
    
    % Group 2 results - 2D algorithms (side by side) / 第二组结果 - 2D算法（左右排布）
    fprintf('\nGroup 2: 2D Algorithm Comparison (MSPDDP vs C-PDDP)\n');
    fprintf('====================================================\n\n');
    
    if results_group2.mspddp.success && results_group2.c_pddp.success
        display_algorithm_comparison_side_by_side('MSPDDP', results_group2.mspddp, 'C-PDDP', results_group2.c_pddp, 2);
    else
        % If one fails, display separately / 如果其中一个失败，分别显示
        if results_group2.mspddp.success
            display_algorithm_detailed_results('MSPDDP', results_group2.mspddp, 2);
        else
            fprintf('MSPDDP algorithm execution failed\n\n');
        end
        
        if results_group2.c_pddp.success
            display_algorithm_detailed_results('C-PDDP', results_group2.c_pddp, 2);
        else
            fprintf('C-PDDP algorithm execution failed\n\n');
        end
    end
    
end

function display_algorithm_comparison_side_by_side(alg1_name, result1, alg2_name, result2, dimension)
    % Side by side display of two algorithms comparison results / 左右排布显示两个算法的比较结果
    
    % Basic performance metrics / 基本性能指标 comparison / 基本性能指标比较
    fprintf('%-20s %-20s %-20s\n', 'Metric', alg1_name, alg2_name);
    fprintf('%-20s %-20s %-20s\n', '------', '--------', '--------');
    fprintf('%-20s %-20.4f %-20.4f\n', 'Computation Time(s)', result1.solve_time, result2.solve_time);
    fprintf('%-20s %-20.6f %-20.6f\n', 'Final Cost', result1.final_cost, result2.final_cost);
    fprintf('%-20s %-20.6f %-20.6f\n', 'Final Error', result1.final_error, result2.final_error);
    fprintf('%-20s %-20.4f %-20.4f\n', 'Trajectory Length', result1.trajectory_length, result2.trajectory_length);
    fprintf('%-20s %-20d %-20d\n', 'Iterations', result1.convergence_iterations, result2.convergence_iterations);
    
    % Waypoint error analysis / 航迹点误差分析
    fprintf('\nWaypoint Error Analysis:\n');
    fprintf('%-20s %-20s %-20s\n', 'Waypoint', alg1_name, alg2_name);
    fprintf('%-20s %-20s %-20s\n', '--------', '--------', '--------');
    
    if strcmp(alg1_name, 'MSPDDP') && strcmp(alg2_name, 'C-PDDP')
        % 2D algorithm waypoint error comparison / 2D算法航迹点误差对比
        analyze_2d_waypoint_errors_side_by_side(result1, result2);
    else
        % 3D algorithm waypoint error comparison / 3D算法航迹点误差对比
        analyze_3d_waypoint_errors_side_by_side(result1, result2, alg1_name, alg2_name);
    end
    
    % Cost function convergence analysis / 代价函数收敛分析
    fprintf('\nCost Function Convergence Analysis:\n');
    fprintf('%-20s %-20s %-20s\n', 'Metric', alg1_name, alg2_name);
    fprintf('%-20s %-20s %-20s\n', '------', '--------', '--------');
    
    if strcmp(alg1_name, 'MSPDDP')
        % MSPDDP cost analysis / MSPDDP代价分析
        total_cost1 = 0;
        for i = 1:length(result1.solution)
            total_cost1 = total_cost1 + calculate_final_cost(result1.solution{i}.J);
        end
        
        % C-PDDP cost analysis / C-PDDP代价分析
        final_cost2 = calculate_final_cost(result2.solution.J);
        
        fprintf('%-20s %-20.6f %-20.6f\n', 'Final Cost', total_cost1, final_cost2);
        fprintf('%-20s %-20.6f %-20.6f\n', 'Initial Cost', result1.solution{1}.J(1), result2.solution.J(1));
    else
        % Other algorithms cost analysis / 其他算法代价分析
        sol1 = result1.solution;
        final_cost1 = calculate_final_cost(sol1.J);
        fprintf('%-20s %-20.6f %-20.6f\n', 'Final Cost', final_cost1, calculate_final_cost(result2.solution.J));
        fprintf('%-20s %-20.6f %-20.6f\n', 'Initial Cost', sol1.J(1), result2.solution.J(1));
    end
    
    % Flight time analysis / 飞行时间分析
    fprintf('\nFlight Time Analysis:\n');
    fprintf('%-20s %-20s %-20s\n', 'Metric', alg1_name, alg2_name);
    fprintf('%-20s %-20s %-20s\n', '------', '--------', '--------');
    
    if strcmp(alg1_name, 'MSPDDP')
        % MSPDDP flight time analysis / MSPDDP飞行时间分析
        % MSPDDP flight time is the sum of the previous non-zero values when Theta becomes 0 in the three segments / MSPDDP的飞行时间是三段轨迹中Theta变为0时的上一个非0值的总和
        total_time1 = 0;
        for i = 1:length(result1.solution)
            sol = result1.solution{i};
            % Use the previous non-zero value when Theta becomes 0 as the optimized flight time for this segment / 使用Theta变为0时的上一个非0值作为该段轨迹的优化后飞行时间
            if isfield(sol, 'Theta') && ~isempty(sol.Theta)
                segment_time = find_theta_convergence_time(sol.Theta);
            else
                % If no Theta, use traditional method calculation / 如果没有Theta，使用传统方法计算
                if iscell(sol.t)
                    segment_time = (length(sol.t) - 1) * 0.1;
                else
                    segment_time = sol.t(end);
                end
            end
            total_time1 = total_time1 + segment_time;
        end
        
        % C-PDDP flight time analysis / C-PDDP飞行时间分析
        sol2 = result2.solution;
        if iscell(sol2.t)
            flight_time2 = (length(sol2.t) - 1) * 0.1;
        else
            flight_time2 = sol2.t(end);
        end
        
        fprintf('%-20s %-20.2f %-20.2f\n', 'Flight Time(s)', total_time1, flight_time2);
    else
        % Other algorithms flight time analysis / 其他算法飞行时间分析
        sol1 = result1.solution;
        if iscell(sol1.t)
            flight_time1 = (length(sol1.t) - 1) * 0.1;
        else
            flight_time1 = sol1.t(end);
        end
        
        sol2 = result2.solution;
        if iscell(sol2.t)
            flight_time2 = (length(sol2.t) - 1) * 0.1;
        else
            flight_time2 = sol2.t(end);
        end
        
        fprintf('%-20s %-20.2f %-20.2f\n', 'Flight Time(s)', flight_time1, flight_time2);
    end
    
    fprintf('\n');
end

function analyze_2d_waypoint_errors_side_by_side(result1, result2)
    % 2D algorithm waypoint error side by side comparison analysis
    
    % MSPDDP waypoints / MSPDDP航迹点
    target_waypoints_mspddp = {[0; 0; 0], [8; 12; pi/3], [23; 18; pi/6], [28; 33; pi/2]};
    waypoint_names = {'Start', 'Waypoint1', 'Waypoint2', 'End'};
    
    % C-PDDP waypoints (original coordinates) / C-PDDP航迹点（原始坐标）
    s_p = 30;
    target_waypoints_cpddp = {[0; 0; 0], [80; 120; 0], [230; 180; 0], [280; 330; pi/2]};
    
    for i = 1:length(waypoint_names)
        if i == 1
            % Start point / 起点
            error1 = 0;  % Start point error is 0
            error2 = 0;  % Start point error is 0
        elseif i == length(waypoint_names)
            % End point / 终点
            if iscell(result1.solution{end}.x)
                final_state1 = result1.solution{end}.x{end};
            else
                final_state1 = result1.solution{end}.x(:, end);
            end
            error1 = norm(final_state1 - target_waypoints_mspddp{i});
            
            if iscell(result2.solution.x)
                x_traj2 = zeros(3, length(result2.solution.x));
                for j = 1:length(result2.solution.x)
                    x_traj2(:, j) = result2.solution.x{j};
                end
            else
                x_traj2 = result2.solution.x;
            end
            x_traj2_scaled = x_traj2 * s_p;
            final_state2 = x_traj2_scaled(:, end);
            error2 = norm(final_state2 - target_waypoints_cpddp{i});
        else
            % Intermediate waypoints / 中间航点
            if iscell(result1.solution{i}.x)
                final_state1 = result1.solution{i}.x{end};
            else
                final_state1 = result1.solution{i}.x(:, end);
            end
            error1 = norm(final_state1 - target_waypoints_mspddp{i});
            
            if iscell(result2.solution.x)
                x_traj2 = zeros(3, length(result2.solution.x));
                for j = 1:length(result2.solution.x)
                    x_traj2(:, j) = result2.solution.x{j};
                end
            else
                x_traj2 = result2.solution.x;
            end
            x_traj2_scaled = x_traj2 * s_p;
            distances = sqrt(sum((x_traj2_scaled(1:2, :) - target_waypoints_cpddp{i}(1:2)).^2, 1));
            [~, closest_idx] = min(distances);
            actual2 = x_traj2_scaled(:, closest_idx);
            error2 = norm(actual2 - target_waypoints_cpddp{i});
        end
        
        fprintf('%-20s %-20.6f %-20.6f\n', waypoint_names{i}, error1, error2);
    end
end

function analyze_3d_waypoint_errors_side_by_side(result1, result2, alg1_name, alg2_name)
    % 3D algorithm waypoint error side by side comparison analysis
    
    target = [0.0; 0.0; 0.0; 300; -60/180*pi; 225/180*pi];
    
    if iscell(result1.solution.x)
        final_state1 = result1.solution.x{end};
    else
        final_state1 = result1.solution.x(:, end);
    end
    error1 = norm(final_state1 - target);
    
    if iscell(result2.solution.x)
        final_state2 = result2.solution.x{end};
    else
        final_state2 = result2.solution.x(:, end);
    end
    error2 = norm(final_state2 - target);
    
    fprintf('%-20s %-20.6f %-20.6f\n', 'End Point Error', error1, error2);
end

function display_algorithm_detailed_results(algorithm_name, result, dimension)
    % Display detailed results of single algorithm / 显示单个算法的详细结果
    
    fprintf('%s algorithm detailed results:\n', algorithm_name);
    fprintf('-------------------\n');
    
    % Basic performance metrics / 基本性能指标
    fprintf('1. Basic Performance Metrics:\n');
    fprintf('   Computation Time: %.4f seconds\n', result.solve_time);
    fprintf('   Final Cost: %.6f\n', result.final_cost);
    fprintf('   Final Error: %.6f\n', result.final_error);
    fprintf('   Trajectory Length: %.4f\n', result.trajectory_length);
    fprintf('   Convergence Iterations: %d\n', result.convergence_iterations);
    
    % Waypoint error analysis / 航迹点误差分析
    fprintf('\n2. Waypoint Error Analysis:\n');
    if strcmp(algorithm_name, 'MSPDDP')
        % MSPDDP has three segments / MSPDDP有三段轨迹
        analyze_mspddp_waypoint_errors(result);
    elseif strcmp(algorithm_name, 'C-PDDP')
        % C-PDDP has intermediate waypoints / C-PDDP有中间航点
        analyze_cpddp_waypoint_errors(result);
    else
        % 3D algorithms only have start and end points
        analyze_3d_waypoint_errors(result, algorithm_name);
    end
    
    % Cost function convergence analysis / 代价函数收敛分析
    fprintf('\n3. Cost Function Convergence Analysis:\n');
    analyze_cost_convergence(result, algorithm_name);
    
    % Flight time analysis / 飞行时间分析
    fprintf('\n4. Flight Time Analysis:\n');
    analyze_flight_time(result, algorithm_name);
    
    fprintf('\n');
end

function analyze_mspddp_waypoint_errors(result)
    % Analyze MSPDDP waypoint errors / 分析MSPDDP的航迹点误差
    
    % Target waypoints / 目标航迹点
    target_waypoints = {[0; 0; 0], [8; 12; pi/3], [23; 18; pi/6], [28; 33; pi/2]};
    waypoint_names = {'Start', 'Waypoint1', 'Waypoint2', 'End'};
    
    for i = 1:length(result.solution)
        sol = result.solution{i};
        target = target_waypoints{i+1};  % Each segment's end point is the target
        
        if iscell(sol.x)
            final_state = sol.x{end};
        else
            final_state = sol.x(:, end);
        end
        
        error = norm(final_state - target);
        fprintf('   %s error: %.6f\n', waypoint_names{i+1}, error);
    end
end

function analyze_cpddp_waypoint_errors(result)
    % Analyze C-PDDP waypoint errors / 分析C-PDDP的航迹点误差
    
    % Target waypoints / 目标航迹点 (original coordinates)
    s_p = 30;
    target_waypoints = {[0; 0; 0], [80; 120; 0], [230; 180; 0], [280; 330; pi/2]};
    waypoint_names = {'Start', 'Intermediate Waypoint1', 'Intermediate Waypoint2', 'End'};
    
    if iscell(result.solution.x)
        x_traj = zeros(3, length(result.solution.x));
        for i = 1:length(result.solution.x)
            x_traj(:, i) = result.solution.x{i};
        end
    else
        x_traj = result.solution.x;
    end
    
    % Scale trajectory back to original scale / 将轨迹缩放回原始尺度
    x_traj_scaled = x_traj * s_p;
    
    % Calculate error for each waypoint / 计算每个航迹点的误差
    for i = 1:length(target_waypoints)
        target = target_waypoints{i};
        
        if i == 1
            % Start point / 起点
            actual = x_traj_scaled(:, 1);
        elseif i == length(target_waypoints)
            % End point / 终点
            actual = x_traj_scaled(:, end);
        else
            % Intermediate waypoints / 中间航点 - find closest point
            distances = sqrt(sum((x_traj_scaled(1:2, :) - target(1:2)).^2, 1));
            [~, closest_idx] = min(distances);
            actual = x_traj_scaled(:, closest_idx);
        end
        
        error = norm(actual - target);
        fprintf('   %s error: %.6f (original coordinates)\n', waypoint_names{i}, error);
    end
end

function analyze_3d_waypoint_errors(result, algorithm_name)
    % Analyze 3D algorithm waypoint errors / 分析3D算法航迹点误差

        target = [0.0; 0.0; 0.0; 300; -60/180*pi; 225/180*pi];

    if iscell(result.solution.x)
        final_state = result.solution.x{end};
    else
        final_state = result.solution.x(:, end);
    end
    
    fprintf('   End point error: %.6f\n', norm(final_state - target));
end

function analyze_cost_convergence(result, algorithm_name)
    % Analyze cost function convergence / 分析代价函数收敛性
    
    if strcmp(algorithm_name, 'MSPDDP')
        % MSPDDP has three segments / MSPDDP有三段轨迹 cost / 代价
        total_cost = 0;
        for i = 1:length(result.solution)
            sol = result.solution{i};
            total_cost = total_cost + calculate_final_cost(sol.J);
        end
        fprintf('   Total cost: %.6f\n', total_cost);
        fprintf('   Segment costs: ');
        for i = 1:length(result.solution)
            fprintf('%.6f ', calculate_final_cost(result.solution{i}.J));
        end
        fprintf('\n');
    else
        % Other algorithms / 其他算法
        sol = result.solution;
        
        final_cost = calculate_final_cost(sol.J);
        fprintf('   Final cost: %.6f\n', final_cost);
        fprintf('   Initial cost: %.6f\n', sol.J(1));
        fprintf('   Cost improvement: %.6f (%.2f%%)\n', sol.J(1) - final_cost, (sol.J(1) - final_cost)/sol.J(1)*100);
    end
end

function analyze_flight_time(result, algorithm_name)
    % Analyze flight time / 分析飞行时间
    
    if strcmp(algorithm_name, 'MSPDDP')
        % MSPDDP has three segments / MSPDDP有三段轨迹
        total_time = 0;
        fprintf('   Segment flight times: ');
        for i = 1:length(result.solution)
            sol = result.solution{i};
            if iscell(sol.t)
                % If t is cell array, calculate time steps multiplied by time step size / 如果t是元胞数组，计算时间步数乘以时间步长
                segment_time = (length(sol.t) - 1) * 0.1;  % dt = 0.1
            else
                % If t is array, take the last time value / 如果t是数组，取最后一个时间值
                segment_time = sol.t(end);
            end
            total_time = total_time + segment_time;
            fprintf('%.2f ', segment_time);
        end
        fprintf('seconds\n');
        fprintf('   Total flight time: %.2f seconds\n', total_time);
    else
        % Other algorithms / 其他算法
        sol = result.solution;
        
        if iscell(sol.t)
            % If t is cell array, calculate time steps multiplied by time step size / 如果t是元胞数组，计算时间步数乘以时间步长
            flight_time = (length(sol.t) - 1) * 0.1;  % Assume dt = 0.1
        else
            % If t is array, take the last time value / 如果t是数组，取最后一个时间值
            flight_time = sol.t(end);
        end
        fprintf('   Flight time: %.2f seconds\n', flight_time);
    end
end

function plot_trajectory_comparison(results_group1, results_group2)
    % Plot trajectory comparison, including waypoints and obstacles / 绘制轨迹比较图，包括航迹点和障碍物
    
    fprintf('\nPlotting trajectory comparison...\n');
    
    % Create 3D trajectory comparison figure / 创建3D轨迹比较图
    figure('Name', '3D Algorithm Trajectory Comparison', 'Position', [100, 100, 1200, 500]);
    
    % 3D trajectory comparison / 3D轨迹比较
    subplot(1, 2, 1);
    if results_group1.pddp.success && results_group1.fft_cddp.success
        % Draw obstacles / 绘制障碍物
        s_p = 200;
        x_0 = [5000.0/s_p; 5000.0/s_p; 5000.0/s_p; 300.0; 0/180*pi; 180/180*pi];
        x_star = [0.0; 0.0; 0.0; 300; -60/180*pi; 225/180*pi];
        
        % Obstacle positions / 障碍物位置
        center1 = [1500/s_p, 2000/s_p]; r1 = 500/s_p;
        center2 = [2800/s_p, 4800/s_p]; r2 = 500/s_p;
        
        % Draw cylinder obstacles / 绘制圆柱体障碍物
        [hCylinder1, hEndPlate1_1, hEndPlate1_2] = cylinder3([center1(1) center1(2) x_0(3)]*s_p, ...
            [center1(1) center1(2) x_star(3)]*s_p, r1*s_p, 1000, 'r', 1, 0);
        [hCylinder2, hEndPlate2_1, hEndPlate2_2] = cylinder3([center2(1) center2(2) x_0(3)]*s_p, ...
            [center2(1) center2(2) x_star(3)]*s_p, r2*s_p, 1000, 'r', 1, 0);
        
        % Draw PDDP trajectory / 绘制PDDP轨迹
        if iscell(results_group1.pddp.solution.x)
            x_traj_pddp = zeros(6, length(results_group1.pddp.solution.x));
            for i = 1:length(results_group1.pddp.solution.x)
                x_traj_pddp(:, i) = results_group1.pddp.solution.x{i};
            end
        else
            x_traj_pddp = results_group1.pddp.solution.x;
        end
        
        % Draw FFT-CDDP trajectory / 绘制FFT-CDDP轨迹
        if iscell(results_group1.fft_cddp.solution.x)
            x_traj_fft = zeros(6, length(results_group1.fft_cddp.solution.x));
            for i = 1:length(results_group1.fft_cddp.solution.x)
                x_traj_fft(:, i) = results_group1.fft_cddp.solution.x{i};
            end
        else
            x_traj_fft = results_group1.fft_cddp.solution.x;
        end
        
        % Draw trajectories / 绘制轨迹
        h_traj_pddp = plot3(x_traj_pddp(1, :)*s_p, x_traj_pddp(2, :)*s_p, x_traj_pddp(3, :)*s_p, 'b-', 'LineWidth', 2);
        hold on;
        h_traj_fft = plot3(x_traj_fft(1, :)*s_p, x_traj_fft(2, :)*s_p, x_traj_fft(3, :)*s_p, 'r--', 'LineWidth', 2);
        
        % Draw start and end points / 绘制起点和终点
        h_start = plot3(x_0(1)*s_p, x_0(2)*s_p, x_0(3)*s_p, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        h_end = plot3(x_star(1)*s_p, x_star(2)*s_p, x_star(3)*s_p, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('3D Trajectory Comparison');
        legend([hCylinder1, h_traj_pddp, h_traj_fft, h_start, h_end], 'Obstacles', 'PDDP', 'FFT-CDDP', 'Start', 'End', 'Location', 'best');
        grid on;
        view(45, 30);
    end
    
    % 2D trajectory comparison / 2D轨迹比较
    subplot(1, 2, 2);
    if results_group2.mspddp.success && results_group2.c_pddp.success
        % Draw MSPDDP trajectory (three segments) / 绘制MSPDDP轨迹（三段）
        colors = {'b-', 'g-', 'c-'};
        for i = 1:length(results_group2.mspddp.solution)
            sol = results_group2.mspddp.solution{i};
            if iscell(sol.x)
                x_traj = zeros(3, length(sol.x));
                for j = 1:length(sol.x)
                    x_traj(:, j) = sol.x{j};
                end
            else
                x_traj = sol.x;
            end
            plot(x_traj(1, :), x_traj(2, :), colors{i}, 'LineWidth', 2);
            hold on;
        end
        
        % Draw C-PDDP trajectory (divided by 10 to match MSPDDP trajectory size) / 绘制C-PDDP轨迹（除以10以匹配MSPDDP轨迹大小）
        if iscell(results_group2.c_pddp.solution.x)
            x_traj_cpddp = zeros(3, length(results_group2.c_pddp.solution.x));
            for i = 1:length(results_group2.c_pddp.solution.x)
                x_traj_cpddp(:, i) = results_group2.c_pddp.solution.x{i};
            end
        else
            x_traj_cpddp = results_group2.c_pddp.solution.x;
        end
        
        % Scale C-PDDP trajectory back to original scale, then divide by 10 for comparison with MSPDDP / 将C-PDDP轨迹缩放回原始尺度，然后除以10以便与MSPDDP比较
        s_p = 30;
        x_traj_cpddp_scaled = x_traj_cpddp * s_p / 10;  % Scale back to original scale, then divide by 10
        plot(x_traj_cpddp_scaled(1, :), x_traj_cpddp_scaled(2, :), 'r--', 'LineWidth', 2);
        
        % Draw waypoints / 绘制航迹点
        % MSPDDP waypoints / MSPDDP航迹点
        plot(0, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        plot(8, 12, 'mo', 'MarkerSize', 8, 'LineWidth', 2);
        plot(23, 18, 'mo', 'MarkerSize', 8, 'LineWidth', 2);
        plot(28, 33, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        
        xlabel('X (m)'); ylabel('Y (m)');
        title('2D Algorithm Trajectory Comparison');
        legend('MSPDDP-1', 'MSPDDP-2', 'MSPDDP-3', 'C-PDDP', 'Start', 'Waypoint1', 'Waypoint2', 'End', 'Location', 'best');
        grid on;
        axis equal;
    end
    
    fprintf('Trajectory comparison plot completed!\n');
end

%% Helper function: Draw 3D cylinder obstacles / 辅助函数：绘制3D圆柱体障碍物
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
    
    % Draw two circles for closed cylinder / 为封闭圆柱体绘制两个圆
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

