%% Flexible Final Time Constrained Differential Dynamic Programming Algorithm / 灵活终端时间约束差分动态规划算法
% Obstacle avoidance (×), Waypoints (√) / 避障 (×), 航点 (√)

% Performs Differential Dynamic Programming Algorithm to produce a / 执行差分动态规划算法以产生
% locally optimal control sequence. / 局部最优控制序列
%
% Inputs / 输入参数
% 
% x_0      : initial state vector / 初始状态向量
% x_star   : target state vector / 目标状态向量
% t_f      : time horizon / 时间范围
% Interval : number of discretizations of time / 时间离散化数量
% dyn      : instance of Dynamics class / 动力学类实例
% cost     : instance of Cost class / 成本类实例
% constraints : constraint functions / 约束函数
% u_max    : maximum magnitude of control, used for clamping / 控制的最大幅度，用于限制
% num_iter : number of DDP iterations / DDP迭代次数
% alpha    : DDP learning parameter for line searching / DDP线搜索学习参数
%
% Outputs / 输出参数
% 
% sol : structure with solution components / 包含解组件的结构体
%
function sol = penalty_free_time_ddp(x_0, x_star, t_f, Interval, dyn, cost, constraints, u_max, num_iter, alpha)
    %% Allocate arrays for DDP / 为DDP分配数组

    % Number of states along trajectory / 轨迹上的状态数量
    N = Interval +1;
    
    % Time stamp array and timestep / 时间戳数组和时间步长
    t = linspace(0.0, t_f, N); % Create time vector from 0 to t_f with N points / 创建从0到t_f的N个点的时间向量
    dt = t(2) - t(1); % Calculate time step / 计算时间步长
    t_max = 55; % Maximum terminal time / 最大终端时间
    t_min = 15; % Minimum terminal time / 最小终端时间
    
    % Cost history / 成本历史
    J = zeros(num_iter, 1); % Initialize cost array / 初始化成本数组
    
    % Control energy history / 控制能量历史
    E = zeros(num_iter, 1); % Initialize energy array / 初始化能量数组
    
    % Terminal time history / 终端时间历史
    Tf = zeros(num_iter, 1); % Initialize terminal time array / 初始化终端时间数组
    J_accepted = zeros(num_iter, 1); % Strict best-so-far nominal costs
    accepted_iteration_indices = zeros(num_iter, 1);
    accepted_count = 0;
    best_cost = Inf;
    best_x = {};
    best_u = {};
    best_t = t;
    
    % State trajectory / 状态轨迹
    x = cell(N, 1); % Current state trajectory / 当前状态轨迹
    x_new = cell(N, 1); % Updated state trajectory / 更新的状态轨迹
    x{1} = x_0; % Set initial state / 设置初始状态
    x_new{1} = x_0; % Set initial state for new trajectory / 为新轨迹设置初始状态
    
    % Control Input trajectory / 控制输入轨迹
    u = cell(N, 1); % Control sequence / 控制序列
    
    % Value function and derivatives / 价值函数及其导数
    V = zeros(N, 1); % Value function / 价值函数
    V_x = cell(N, 1); % Value function gradient w.r.t. state / 价值函数对状态的梯度
    V_xx = cell(N, 1); % Value function Hessian w.r.t. state / 价值函数对状态的Hessian矩阵
    V_t = cell(1, 1); % Value function derivative w.r.t. time / 价值函数对时间的导数
    V_tx = cell(1, 1); % Value function mixed derivative w.r.t. time and state / 价值函数对时间和状态的混合导数
    V_tt = cell(1, 1); % Value function second derivative w.r.t. time / 价值函数对时间的二阶导数
    V_p = cell(1, 1); % Projected value function gradient / 投影价值函数梯度
    V_pp = cell(1, 1); % Projected value function Hessian / 投影价值函数Hessian矩阵
    
    % State action value derivatives / 状态动作价值导数
    Q_x = cell(N, 1); % Q-function gradient w.r.t. state / Q函数对状态的梯度
    Q_u = cell(N, 1); % Q-function gradient w.r.t. control / Q函数对控制的梯度
    Q_xx = cell(N, 1); % Q-function Hessian w.r.t. state / Q函数对状态的Hessian矩阵
    Q_uu = cell(N, 1); % Q-function Hessian w.r.t. control / Q函数对控制的Hessian矩阵
    Q_xu = cell(N, 1); % Q-function mixed derivative w.r.t. state and control / Q函数对状态和控制的混合导数
    Q_ux = cell(N, 1); % Q-function mixed derivative w.r.t. control and state / Q函数对控制和状态的混合导数
    
    % Constraints derivatives / 约束导数
    Dx = cell(N, length(constraints)); % Constraint gradients w.r.t. state / 约束对状态的梯度
    lambda = cell(N, length(constraints)); % Lagrange multipliers / 拉格朗日乘数

    %% Initialize DDP with a random input sequence / 用随机输入序列初始化DDP

    fprintf("initializing with a random control sequence...\n") % Display initialization message / 显示初始化消息
    
    % Generate random control sequence / 生成随机控制序列
    for k = 1:N-1
%         u{k} = 2 .* u_max .* rand(length(u_max), 1) - u_max; % Random control (commented) / 随机控制（已注释）
        u{k} = zeros(numel(u_max), 1); % Initialize with zero control / 用零控制初始化
    end
    u{N} = zeros(numel(u_max), 1); % Set terminal control to zero / 将终端控制设置为零
%     [path, u_PN] = Proportional_guidance(x_0, x_star, dyn, t_f, dt); % Proportional guidance (commented) / 比例制导（已注释）
%     for k = 1:N-1
%         u{k} = u_PN(k); % Use proportional guidance control (commented) / 使用比例制导控制（已注释）
%     end
    
    fprintf("generating initial trajectory...\n") % Display trajectory generation message / 显示轨迹生成消息
    
    % Generate initial trajectory using random control sequence / 使用随机控制序列生成初始轨迹
    for k = 1:N-1
        x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}) .* dt; % Integrate dynamics forward / 前向积分动力学
    end
    
    %% Perform main DDP iterations on the trajectory and input sequence / 对轨迹和输入序列执行主要DDP迭代
    
    fprintf("beginning DDP...\n") % Display DDP start message / 显示DDP开始消息
    
    sigma = 1; % Penalty parameter for constraints / 约束的惩罚参数
    for k = 1:N-1 % Initialize Lagrange multipliers / 初始化拉格朗日乘数
        for constr = 1:length(constraints)
            lambda{k,constr} = 0.01; % Set initial multiplier value / 设置初始乘数值
        end
    end
    
    converged=false;
    lastFinite=[];
    numerical=struct('linear_solves',0,'regularized_solves',0,'maximum_damping',0,'backward_retries',0);
    for i = 1:num_iter % Main DDP iteration loop / 主要DDP迭代循环
        fprintf("DDP iteration %d out of %d...\n", i, num_iter); % Display iteration progress / 显示迭代进度
        
        t = linspace(0.0, t_f, N); % Update time vector / 更新时间向量
        dt = t(2) - t(1); % Recalculate time step / 重新计算时间步长
        
        % Update the current trajectory / 更新当前轨迹
        x = x_new; % Copy new trajectory to current / 将新轨迹复制到当前
        saved=struct('x',{x},'u',{u},'t',t,'J',J(1:max(1,i-1)),'iter',i,'final_time',t_f);
        try
        
        % Compute total cost / 计算总成本
        J(i) = cost.phi(x{N}, x_star); % Terminal cost / 终端成本
        for k = 1:N-1
            J(i) = J(i) + cost.L(x{k}, x_star, u{k}, dt) + dt; % Running cost + time penalty / 运行成本 + 时间惩罚
        end

        % Keep trial costs separate from strict best-so-far accepted iterates.
        if J(i) < best_cost
            best_cost = J(i);
            best_x = x;
            best_u = u;
            best_t = t;
            accepted_count = accepted_count + 1;
            J_accepted(accepted_count) = J(i);
            accepted_iteration_indices(accepted_count) = i;
        end
        
        saved.J=J(1:i);
        if ~isfinite(J(i)),error('ddpack:numerics:NonfiniteTrajectory','Nominal objective is nonfinite.');end
        if ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),x)) || ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),u))
            error('ddpack:numerics:NonfiniteTrajectory','Nominal trajectory is nonfinite.');
        end
        lastFinite=saved;
        % Compute control energy usage / 计算控制能量使用
        for k = 1:N-1
            E(i) = E(i) + 0.5 .* u{k}.' * u{k} .* dt; % Accumulate control energy / 累积控制能量
        end
        
        % Log terminal time / 记录终端时间
        Tf(i) = t_f; % Store current terminal time / 存储当前终端时间
        
        if i > 1 && abs(J(i) - J(i-1)) < 1e-3 % Check for convergence / 检查收敛性
            fprintf("Convergence !!! ...\n") % Display convergence message / 显示收敛消息
            u{N} = u{N-1}; % Set terminal control / 设置终端控制
            iter = i; % Store iteration number / 存储迭代次数
            converged=true;break; % Exit iteration loop / 退出迭代循环
        end
        
        % Compute terminal value function and derivatives / 计算终端价值函数及其导数
        V(N) = cost.phi(x{N}, x_star); % Terminal value function / 终端价值函数
        V_x{N} = cost.phi_x(x{N}, x_star); % Terminal value gradient / 终端价值梯度
        V_xx{N} = cost.phi_xx(x{N}, x_star); % Terminal value Hessian / 终端价值Hessian矩阵
        u{N} = u{N-1}; % Set terminal control / 设置终端控制
        V_t = V_x{N}' * dyn.F(x{N}, u{N}) + 1 * dt; % Terminal value time derivative / 终端价值时间导数
        V_tx = ( V_xx{N} * dyn.F(x{N}, u{N}) )' + V_x{N}' * ( dyn.Phi(x{N}, u{N}, dt) - eye(numel(x{N})) ) ./ dt; % Mixed derivative / 混合导数
        V_tt = V_tx * dyn.F(x{N}, u{N}); % Second time derivative / 二阶时间导数
        
        
        % Perform backwards pass / 执行反向传递
        for k = N-1:-1:1 % Backward iteration through time / 时间反向迭代
            % Compute state-action value function derivatives / 计算状态动作价值函数导数
            if k == N-1 % Special case for second-to-last time step / 倒数第二个时间步的特殊情况
                V_p = V_x{N}' - V_t' * ddpack.checkedTimeSolve(V_tt,V_tx); % Projected value gradient / 投影价值梯度
                V_pp = V_xx{N} - V_tx' * ddpack.checkedTimeSolve(V_tt,V_tx); % Projected value Hessian / 投影价值Hessian矩阵
                Q_x{N-1} = cost.L_x(x{N-1}, x_star, u{N-1}, dt) + dyn.Phi(x{N-1}, u{N-1}, dt)' * V_p'; % Q-function state gradient / Q函数状态梯度
                Q_u{N-1} = cost.L_u(x{N-1}, u{N-1}, dt) + dyn.Beta(x{N-1}, u{N-1}, dt)' * V_p'; % Q-function control gradient / Q函数控制梯度
                Q_xx{N-1} = cost.L_xx(x{N-1}, x_star, u{N-1}, dt) + dyn.Phi(x{N-1}, u{N-1}, dt)' * V_pp * dyn.Phi(x{N-1}, u{N-1}, dt); % Q-function state Hessian / Q函数状态Hessian矩阵
                Q_uu{N-1} = cost.L_uu(x{N-1}, u{N-1}, dt) + dyn.Beta(x{N-1}, u{N-1}, dt)' * V_pp * dyn.Beta(x{N-1}, u{N-1}, dt); % Q-function control Hessian / Q函数控制Hessian矩阵
                Q_xu{N-1} = cost.L_xu(x{N-1}, u{N-1}, dt) + dyn.Phi(x{N-1}, u{N-1}, dt)' * V_pp * dyn.Beta(x{N-1}, u{N-1}, dt); % Q-function mixed derivative / Q函数混合导数
                Q_ux{N-1} = cost.L_ux(x{N-1}, u{N-1}, dt) + dyn.Beta(x{N-1}, u{N-1}, dt)' * V_pp * dyn.Phi(x{N-1}, u{N-1}, dt); % Q-function mixed derivative / Q函数混合导数
            else % Standard case for other time steps / 其他时间步的标准情况
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, dt) + dyn.Phi(x{k}, u{k}, dt)' * V_x{k+1}; % Q-function state gradient / Q函数状态梯度
                Q_u{k} = cost.L_u(x{k}, u{k}, dt) + dyn.Beta(x{k}, u{k}, dt)' * V_x{k+1}; % Q-function control gradient / Q函数控制梯度
                Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, dt) + dyn.Phi(x{k}, u{k}, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, dt); % Q-function state Hessian / Q函数状态Hessian矩阵
                Q_uu{k} = cost.L_uu(x{k}, u{k}, dt) + dyn.Beta(x{k}, u{k}, dt)' * V_xx{k+1} * dyn.Beta(x{k}, u{k}, dt); % Q-function control Hessian / Q函数控制Hessian矩阵
                Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + dyn.Phi(x{k}, u{k}, dt)' * V_xx{k+1} * dyn.Beta(x{k}, u{k}, dt); % Q-function mixed derivative / Q函数混合导数
                Q_ux{k} = cost.L_ux(x{k}, u{k}, dt) + dyn.Beta(x{k}, u{k}, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, dt); % Q-function mixed derivative / Q函数混合导数
            end
            
            % construct equality quadratic programming problem / 构造等式二次规划问题
            for constr = 1:length(constraints) % Loop through all constraints / 遍历所有约束
                G_value =  constraints(constr).G(x{k}, u{k}, dt) ; % Evaluate constraint function / 评估约束函数
                if G_value < 0.00001 % Check if constraint is active / 检查约束是否激活
                    Dx{k,constr} = constraints(constr).G_x(x{k}, u{k}, dt); % Compute constraint gradient / 计算约束梯度
%                     lambda{k,constr} = lambda{k,constr} + sigma * G_value; % Update Lagrange multiplier (commented) / 更新拉格朗日乘数（已注释）
                    Q_x{k} = Q_x{k} + 0.004 * Dx{k,constr}' + G_value * sigma * Dx{k,constr}'; % Add constraint penalty to Q_x / 向Q_x添加约束惩罚
                    Q_xx{k} = Q_xx{k} + Dx{k,constr}'* sigma *Dx{k,constr}; % Add constraint penalty to Q_xx / 向Q_xx添加约束惩罚
%                     Q_xx{k} = Q_xx{k} + Dx{k,constr}'* sigma *Dx{k,constr} + lambda_temp * 2*dt + G_value * sigma * 2*dt; % Additional penalty (commented) / 额外惩罚（已注释）
                end
            end
            
            
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},[Q_u{k},Q_ux{k}],numerical);
            penaltyFeed{k}=-gains(:,1);penaltyFeedback{k}=-gains(:,2:end);
            % Compute the value function derivatives / 计算价值函数导数
            V_x{k} = Q_x{k} - Q_xu{k} * (-penaltyFeed{k}); % Value function gradient / 价值函数梯度
            V_xx{k} = Q_xx{k} - Q_xu{k} * (-penaltyFeedback{k}); % Value function Hessian / 价值函数Hessian矩阵
            
        end
        
        sigma = 1.15 * sigma; % Increase penalty parameter / 增加惩罚参数
        
        % Update control sequence from previous iteration / 从上次迭代更新控制序列
        for k = 1:N-1 % Forward iteration through time / 时间前向迭代
            % Compute control update feed-forward and feed-back / 计算控制更新前馈和反馈
            du_ff = penaltyFeed{k}; % Feed-forward control update / 前馈控制更新
            du_fb = penaltyFeedback{k} * (x_new{k} - x{k}); % Feed-back control update / 反馈控制更新

            % Limit feed forward control modification with clamping / 用限制限制前馈控制修改
            du = alpha .* (du_ff + du_fb);
            if any(~isfinite(du(:))),error('ddpack:numerics:NonfiniteTrajectory','Control update is nonfinite.');end
            % % Scale control update / 缩放控制更新
            for m = 1:numel(u_max) % Apply control limits / 应用控制限制
%                     du_ff(m) = min(u_max(m), max(-u_max(m), du_ff(m) + u{k}(m))) - u{k}(m); % Clamp feed-forward (commented) / 限制前馈（已注释）
                du(m) = min(u_max(m), max(-u_max(m), du(m) + u{k}(m))) - u{k}(m); % Clamp control update / 限制控制更新
%                     du(m) = min(u_max(m)/5, max(-u_max(m)/5, du(m))); % Alternative clamping (commented) / 替代限制（已注释）
            end

            % Update control / 更新控制
            u{k} = u{k} + du; % Apply control update / 应用控制更新

            % Compute next state in trajectory with new control / 用新控制计算轨迹中的下一个状态
            x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}) .* dt; % Integrate dynamics / 积分动力学

            % Return error if problem with trajectory / 如果轨迹有问题则返回错误
            if any(~isfinite(x_new{k+1}(:))) || any(~isfinite(u{k}(:)))
                error('ddpack:numerics:NonfiniteTrajectory','Candidate rollout is nonfinite.');
            end
        end

        % Update terminal time / 更新终端时间
        % Compute terminal time update feed-forward and feed-back / 计算终端时间更新前馈和反馈
        dtf_ff = ddpack.checkedTimeSolve(V_tt,-V_t); % Feed-forward terminal time update / 前馈终端时间更新
        dtf_fb = ddpack.checkedTimeSolve(V_tt,-V_tx) * (x_new{N} - x{N}); % Feed-back terminal time update / 反馈终端时间更新

        % Limit feed forward control modification with clamping / 用限制限制前馈控制修改
%             dtf_ff = min(20, max(10, dtf_ff + t_f)) - t_f; % Clamp terminal time (commented) / 限制终端时间（已注释）

        % Update terminal time / 更新终端时间
        dtf = 0.4 * ( dtf_ff + dtf_fb ); % Scale terminal time update / 缩放终端时间更新
        t_f = min(t_max, max(t_min, dtf + t_f)); % Apply terminal time limits / 应用终端时间限制

        % Return error if problem with trajectory / 如果轨迹有问题则返回错误
        if N <= 0 % Check for invalid discretization / 检查无效离散化
            sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, 1); % Assemble error solution / 组装错误解
            fprintf("Problem with terminal time ...\n") % Display error message / 显示错误消息
            return % Exit function / 退出函数
        end
        
        catch exception
            if isempty(lastFinite),rethrow(exception);end
            saved=lastFinite;
            if accepted_count>0
                saved.x=best_x;saved.u=best_u;saved.t=best_t;saved.final_time=best_t(end);saved.J=J_accepted(1:accepted_count);
            end
            sol=ddpack.numericalFailureSolution(saved,exception,numerical);
            return;
        end
    end
    
    %% Assemble and return solution structure / 组装并返回解结构
    fprintf("finished DDP, assembling results for post-processing...\n"); % Display completion message / 显示完成消息
    
    % Assemble solution / 组装解
    if ~exist('iter', 'var') % Check if iteration variable exists / 检查迭代变量是否存在
        iter = num_iter; % Set to maximum iterations / 设置为最大迭代次数
    end
    sol = assemble_solution(best_x, best_u, best_t, J, E, Tf, Q_u, Q_uu, Q_ux, 0, iter); % Return best accepted iterate
    sol.J_trial = J;
    sol.J_accepted = J_accepted(1:accepted_count);
    sol.accepted_iteration_indices = accepted_iteration_indices(1:accepted_count);
    sol.returned_iteration = accepted_iteration_indices(accepted_count);
    sol.cost_definition = 'terminal + running + time-step penalty';
    finite_trials = J(isfinite(J) & J ~= 0);
    [verified_accepted, verified_indices] = ...
        ddpack.strictAcceptedCosts(finite_trials);
    if ~isequal(verified_accepted, sol.J_accepted)
        error('ddpack:fft:AcceptedHistoryMismatch', ...
            'Tracked accepted costs do not match strict best-so-far semantics.');
    end
    sol.accepted_iteration_indices = verified_indices;
    sol.returned_iteration = verified_indices(end);
    sol.initial_cost = finite_trials(1);
    sol.best_accepted_cost = best_cost;
    sol.returned_cost = best_cost;
    sol.accepted_count = accepted_count;
    sol.rejected_count = numel(finite_trials) - accepted_count;
    if converged
        sol.termination_reason = 'trial_cost_change_below_tolerance';
        sol.convergence_status = 'converged_by_cost_change';
    else
        sol.termination_reason = 'maximum_iterations_reached';
        sol.convergence_status = 'best_accepted_iterate_returned';
    end
    sol.numerical_diagnostics=numerical;
end

%% Helper Functions for DDP Algorithm / DDP算法的辅助函数

% Assembles and returns solution structure / 组装并返回解结构
%
% Inputs / 输入参数
%
% x               : locally optimal state trajectory / 局部最优状态轨迹
% u               : locally optimal control sequence / 局部最优控制序列
% t               : discretized time stamps / 离散化时间戳
% J               : iteration history of cost function / 成本函数的迭代历史
% E               : iteration history of control energy / 控制能量的迭代历史
% Q_u, Q_uu, Q_ux : derivatives of state action value function / 状态动作价值函数的导数
% error           : zero if no error, nonzero else / 无错误时为0，否则非零
%
% Outputs / 输出参数
%
% sol : solution structure / 解结构
function sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, error, iter)

    % Solution structure / 解结构
    sol = struct; % Create solution structure / 创建解结构
    sol.error = error; % Error flag / 错误标志
    sol.x = x; % State trajectory / 状态轨迹
    sol.u = u; % Control sequence / 控制序列
    sol.t = t; % Time vector / 时间向量
    sol.dt = t(2) - t(1); % Time step / 时间步长
    sol.J = J; % Cost history / 成本历史
    sol.E = E; % Energy history / 能量历史
    sol.Tf = Tf; % Terminal time history / 终端时间历史
    sol.Q_u = Q_u; % Q-function control gradients / Q函数控制梯度
    sol.Q_uu = Q_uu; % Q-function control Hessians / Q函数控制Hessian矩阵
    sol.Q_ux = Q_ux; % Q-function mixed derivatives / Q函数混合导数
    sol.iter = iter; % Number of iterations / 迭代次数
    
    return % Return solution / 返回解
end
