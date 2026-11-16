%% Parameterized Differential Dynamic Programming Algorithm / 参数化差分动态规划算法
% Obstacle avoidance (√), Waypoints (×) / 避障 (√), 航点 (×)

% Performs Differential Dynamic Programming Algorithm to produce a locally optimal control sequence / 
% 执行差分动态规划算法以产生局部最优控制序列
%
% Inputs / 输入参数
% 
% x_0      : initial state vector / 初始状态向量
% x_star   : target state vector / 目标状态向量
% t_f      : time horizon / 时间范围
% dt       : time step / 时间步长
% dyn      : instance of Dynamics class / 动力学类实例
% cost     : instance of Cost class / 成本类实例
% constraints : constraint functions / 约束函数
% u_max    : maximum magnitude of control, used for clamping / 控制的最大幅值，用于限制
% num_iter : number of DDP iterations / DDP迭代次数
% alpha    : DDP learning parameter for line searching / DDP线搜索学习参数
%
% Outputs / 输出参数
% 
% sol : structure with solution components / 包含解组件的结构体
%
function sol = free_time_pddp(x_0, x_star, t_f, dt, dyn, cost, constraints, u_max, num_iter, alpha)
    %% Allocate arrays for DDP / 为DDP分配数组

    % Time stamp array and timestep / 时间戳数组和时间步长
    N = floor(t_f ./ dt);  % Number of time discretization points / 时间离散化点数
    t_max = 55;            % Maximum time horizon / 最大时间范围
    t_min = 15;            % Minimum time horizon / 最小时间范围

    % Normalized time vector from 0 to 1 / 从0到1的归一化时间向量
    t = linspace(0.0, 1, N);
    dt = t(2) - t(1);      % Actual time step / 实际时间步长
    
    % Cost history for tracking convergence / 用于跟踪收敛的成本历史
    J = zeros(num_iter, 1);
    
    % Control energy history for analysis / 用于分析的控制能量历史
    E = zeros(num_iter, 1);
    % Time horizon history for tracking / 用于跟踪的时间范围历史
    Tf = zeros(num_iter, 1);
    
    % State trajectory storage / 状态轨迹存储
    x = cell(N, 1);        % Current state trajectory / 当前状态轨迹
    x_new = cell(N, 1);    % Updated state trajectory / 更新的状态轨迹
    x{1} = x_0;            % Initialize with initial state / 用初始状态初始化
    x_new{1} = x_0;        % Initialize with initial state / 用初始状态初始化
    
    % Control Input trajectory storage / 控制输入轨迹存储
    u = cell(N, 1);        % Control sequence / 控制序列
    
    % Value function and derivatives storage / 值函数及其导数存储
    V = zeros(N, 1);       % Value function / 值函数
    V_x = cell(N, 1);      % Value function gradient w.r.t. state / 值函数对状态的梯度
    V_t = cell(N, 1);      % Value function derivative w.r.t. time / 值函数对时间的导数
    V_xx = cell(N, 1);     % Value function Hessian w.r.t. state / 值函数对状态的海塞矩阵
    V_xt = cell(N, 1);     % Value function cross derivative / 值函数交叉导数
    V_tt = cell(N, 1);     % Value function second derivative w.r.t. time / 值函数对时间的二阶导数
    
    % State action value derivatives storage / 状态动作值导数存储
    Q_x = cell(N, 1);      % Q-function gradient w.r.t. state / Q函数对状态的梯度
    Q_u = cell(N, 1);      % Q-function gradient w.r.t. control / Q函数对控制的梯度
    Q_t = cell(N, 1);      % Q-function derivative w.r.t. time / Q函数对时间的导数
    Q_xx = cell(N, 1);     % Q-function Hessian w.r.t. state / Q函数对状态的海塞矩阵
    Q_uu = cell(N, 1);     % Q-function Hessian w.r.t. control / Q函数对控制的海塞矩阵
    Q_tt = cell(N, 1);     % Q-function second derivative w.r.t. time / Q函数对时间的二阶导数
    Q_xu = cell(N, 1);     % Q-function cross derivative w.r.t. state and control / Q函数对状态和控制的交叉导数
    Q_xt = cell(N, 1);     % Q-function cross derivative w.r.t. state and time / Q函数对状态和时间的交叉导数
    Q_ux = cell(N, 1);     % Q-function cross derivative w.r.t. control and state / Q函数对控制和状态的交叉导数
    Q_ut = cell(N, 1);     % Q-function cross derivative w.r.t. control and time / Q函数对控制和时间的交叉导数
	
    % Control policy parameters / 控制策略参数
    kt = cell(N, 1);       % Feed-forward control gain / 前馈控制增益
    Kt = cell(N, 1);       % Feedback control gain matrix / 反馈控制增益矩阵
    Mt = cell(N, 1);       % Time-dependent control gain / 时间相关控制增益

    %% Initialize DDP with a random input sequence / 用随机输入序列初始化DDP

    fprintf("initializing with a random control sequence...\n")
    
    % Generate random control sequence / 生成随机控制序列
    for k = 1:N-1
        % Random control within bounds [-u_max, u_max] / 在边界[-u_max, u_max]内的随机控制
        u{k} = 2 .* (u_max) .* rand(length(u_max), 1) - (u_max);
% 		u{k} = zeros(numel(u_max), 1);  % Alternative: zero control initialization / 替代方案：零控制初始化
    end
    % Final control is zero (terminal condition) / 最终控制为零（终端条件）
    u{N} = zeros(numel(u_max), 1);

%     % Alternative initialization using proportional guidance / 使用比例制导的替代初始化
%     [path, u_PN] = Proportional_guidance3D(x_0, x_star, dyn, t_f, t_f*dt);
%     for k = 1:N-1
%         u{k}(1) = u_PN(1, k);
%         u{k}(2) = u_PN(2, k);
%         u{k} = u{k}';
%     end
    
    fprintf("generating initial trajectory...\n")
    
    % Generate initial trajectory using random control sequence / 使用随机控制序列生成初始轨迹
	for k = 1:N-1
		% Forward integration of dynamics / 动力学前向积分
		x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, t_f) .* dt;
	end
    % Update current trajectory with generated trajectory / 用生成的轨迹更新当前轨迹
    x = x_new;
    
    %% Perform main DDP iterations on the trajectory and input sequence / 对轨迹和输入序列执行主要DDP迭代
    
    fprintf("beginning DDP...\n")

	% Penalty parameter for constraint handling / 约束处理的惩罚参数
	sigma = 1.0;
    
    % Main DDP iteration loop / 主要DDP迭代循环
    converged=false;
    lastFinite=[];
    numerical=struct('linear_solves',0,'regularized_solves',0,'maximum_damping',0,'backward_retries',0);
    for i = 1:num_iter
        saved=struct('x',{x},'u',{u},'t',linspace(0,t_f,N),'J',J(1:max(1,i-1)),'iter',i,'final_time',t_f);
        try
%         fprintf("DDP iteration %d out of %d...\n", i, num_iter);
        
        
        % Compute total cost for current trajectory / 计算当前轨迹的总成本
        J(i) = cost.phi(x{N}, x_star);  % Terminal cost / 终端成本
        for k = 1:N-1
            % Add running cost for each time step / 为每个时间步添加运行成本
            J(i) = J(i) + cost.L(x{k}, x_star, u{k}, t_f, dt);
        end
        
        saved.J=J(1:i);
        if ~isfinite(J(i)),error('ddpack:numerics:NonfiniteTrajectory','Nominal objective is nonfinite.');end
        if ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),x)) || ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),u))
            error('ddpack:numerics:NonfiniteTrajectory','Nominal trajectory is nonfinite.');
        end
        lastFinite=saved;
        % Compute control energy usage for analysis / 计算控制能量使用量用于分析
        for k = 1:N-1
            % Control energy: 0.5 * u' * u * dt / 控制能量：0.5 * u' * u * dt
            E(i) = E(i) + t_f*0.5 .* u{k}.' * u{k} .* dt;
        end
		% Store current time horizon / 存储当前时间范围
		Tf(i) = t_f;
        
        % Check for convergence / 检查收敛性
        if i > 1 && abs(J(i) - J(i-1)) < 1e-5
            fprintf("Convergence !!! ...\n")
            % Set final control to previous control / 将最终控制设置为前一个控制
            u{N} = u{N-1};
            converged=true;break;
        end
        
        % Compute terminal value function and derivatives / 计算终端值函数及其导数
        V(N) = cost.phi(x{N}, x_star);        % Terminal value function / 终端值函数
        V_x{N} = cost.phi_x(x{N}, x_star);    % Terminal value gradient / 终端值梯度
        V_t{N} = cost.phi_t(x{N}, x_star);    % Terminal value time derivative / 终端值时间导数
        V_xx{N} = cost.phi_xx(x{N}, x_star);  % Terminal value Hessian / 终端值海塞矩阵
        V_xt{N} = cost.phi_xt(x{N}, x_star);  % Terminal value cross derivative / 终端值交叉导数
        V_tt{N} = cost.phi_tt(x{N}, x_star);  % Terminal value time second derivative / 终端值时间二阶导数
		
% 		if i>1
% 		end
        
        % Perform backwards pass / 执行反向传播
		for k = N-1:-1:1
    		% Compute state-action value function derivatives / 计算状态动作值函数导数
    		% Q-function gradient w.r.t. state / Q函数对状态的梯度
    		Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1};
    		% Q-function gradient w.r.t. control / Q函数对控制的梯度
    		Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.Beta(x{k}, u{k}, t_f, dt)' * V_x{k+1};
    		% Q-function derivative w.r.t. time / Q函数对时间的导数
    		Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + dyn.theta(x{k}, u{k}, t_f, dt).' * V_x{k+1};
    		% Q-function Hessian w.r.t. state / Q函数对状态的海塞矩阵
    		Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, t_f, dt);
    		% Q-function Hessian w.r.t. control / Q函数对控制的海塞矩阵
    		Q_uu{k} = cost.L_uu(x{k}, u{k}, t_f, dt) + dyn.Beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Beta(x{k}, u{k}, t_f, dt);
    		% Q-function cross derivative w.r.t. state and control / Q函数对状态和控制的交叉导数
    		Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Beta(x{k}, u{k}, t_f, dt);
    		Q_ux{k} = Q_xu{k}';  % Transpose of Q_xu / Q_xu的转置
    		% Q-function cross derivative w.r.t. state and time / Q函数对状态和时间的交叉导数
    		Q_xt{k} = cost.L_xt(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xt{k+1} ...
											 		 + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.theta(x{k}, u{k}, t_f, dt);
    		Q_tx{k} = Q_xt{k}';  % Transpose of Q_xt / Q_xt的转置
    		% Q-function cross derivative w.r.t. control and time / Q函数对控制和时间的交叉导数
    		Q_ut{k} = cost.L_ut(x{k}, u{k}, t_f, dt) + dyn.Beta(x{k}, u{k}, t_f, dt)' * V_xt{k+1} ...
											 		 + dyn.Beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.theta(x{k}, u{k}, t_f, dt);
    		Q_tu{k} = Q_ut{k}';  % Transpose of Q_ut / Q_ut的转置
    		% Q-function second derivative w.r.t. time / Q函数对时间的二阶导数
    		Q_tt{k} = cost.L_tt(x{k}, u{k}, t_f, dt) + V_tt{k+1} + 2*dyn.theta(x{k}, u{k}, t_f, dt)'*V_xt{k+1}...
													 + dyn.theta(x{k}, u{k}, t_f, dt)'*V_xx{k+1} * dyn.theta(x{k}, u{k}, t_f, dt);
            
            % Construct constraint penalty terms for quadratic programming / 为二次规划构建约束惩罚项
            for constr = 1:length(constraints)
                % Evaluate constraint function / 评估约束函数
                G_value =  constraints(constr).G(x{k}, u{k}, t_f*dt) ;
                % Apply penalty if constraint is violated / 如果约束被违反则应用惩罚
                if G_value < 0.00001
                    % Compute constraint gradient / 计算约束梯度
                    Dx{k,constr} = constraints(constr).G_x(x{k}, u{k}, t_f*dt);
%                     lambda{k,constr} = lambda{k,constr} + sigma * G_value;  % Alternative: Lagrange multiplier / 替代方案：拉格朗日乘数
                    % Add penalty terms to Q-function gradients / 向Q函数梯度添加惩罚项
                    Q_x{k} = Q_x{k} + 0.005 * Dx{k,constr}' + G_value * sigma * Dx{k,constr}';
                    % Tuning parameters for different scenarios / 不同场景的调优参数
                    % Q、R-0.1 dt-0.02 lambda-0.011
                    % Q、R-0.1 dt-0.05 lambda-0.006
                    % Q、R-1.0 dt-0.05 lambda-0.04(10-14,18-20) 0.05(16)
                    % Add penalty terms to Q-function Hessian / 向Q函数海塞矩阵添加惩罚项
                    Q_xx{k} = Q_xx{k} + Dx{k,constr}'* sigma *Dx{k,constr};
%                     Q_xx{k} = Q_xx{k} + Dx{k,constr}'* sigma *Dx{k,constr} + lambda_temp * 2*dt + G_value * sigma * 2*dt;
                end
            end
       		
    		% Compute the value function derivatives / 计算值函数导数
    		% Feed-forward control gain / 前馈控制增益
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},-[Q_u{k},Q_ux{k},Q_ut{k}],numerical);
            kt{k}=gains(:,1);Kt{k}=gains(:,2:1+numel(x_0));Mt{k}=gains(:,end);

%     		% Alternative value function update (commented out) / 替代的值函数更新（已注释）
%     		V_x{k} = Q_x{k} - Q_xu{k} * (Q_uu{k} \ Q_u{k});
%     		V_t{k} = Q_t{k} - Q_tu{k} * (Q_uu{k} \ Q_u{k});
%     		V_xx{k} = Q_xx{k} - Q_xu{k} * (Q_uu{k} \ Q_ux{k});
%     		V_xt{k} = Q_xt{k} - Q_xu{k} * (Q_uu{k} \ Q_ut{k});
%     		V_tx{k} = V_xt{k}';
%     		V_tt{k} = Q_tt{k} - Q_tu{k} * (Q_uu{k} \ Q_ut{k});
    		% Update value function derivatives using control gains / 使用控制增益更新值函数导数
    		V_x{k} = Q_x{k} + Kt{k}'*Q_u{k} + Q_xu{k}*kt{k} + Kt{k}'*Q_uu{k}*kt{k};
    		V_t{k} = Q_t{k} + Mt{k}'*Q_u{k} + Q_tu{k}*kt{k} + Mt{k}'*Q_uu{k}*kt{k};
    		V_xx{k} = Q_xx{k} + Q_xu{k}*Kt{k} + Kt{k}'*Q_ux{k} + Kt{k}'*Q_uu{k}*Kt{k};
    		V_xt{k} = Q_xt{k} + Q_xu{k}*Mt{k} + Kt{k}'*Q_ut{k} + Kt{k}'*Q_uu{k}*Mt{k};
    		V_tt{k} = Q_tt{k} + Q_tu{k}*Mt{k} + Mt{k}'*Q_ut{k} + Mt{k}'*Q_uu{k}*Mt{k};
		end
        
        % Update penalty parameter (exponential increase) / 更新惩罚参数（指数增长）
        sigma = 2.0 * sigma;

		% Compute time horizon update using Newton's method / 使用牛顿法计算时间范围更新
		dtheta = ddpack.checkedTimeSolve(V_tt{1},-V_t{1}) ;

		% Update time horizon with bounds / 用边界更新时间范围
		t_f = t_f + dtheta;
        t_f = min(t_max, max(t_min, t_f));

        % Forward pass, Update control sequence from previous iteration / 前向传播，从前一次迭代更新控制序列
		for k = 1:N-1
            % Compute control update components / 计算控制更新组件
            du_ff = kt{k};                                    % Feed-forward control update / 前馈控制更新
            du_fb = Kt{k} * (x_new{k} - x{k});               % Feedback control update / 反馈控制更新
            du_ft = Mt{k} * dtheta;                           % Time-dependent control update / 时间相关控制更新

%                 % Alternative: Limit feed forward control modification with clamping / 替代方案：用限制限制前馈控制修改
%             for m = 1:numel(u_max)
%                 du_ff(m) = min(u_max(m), max(-u_max(m), ...
%                                      du_ff(m) + u{k}(m))) - u{k}(m);
%             end

            % Update control with line search parameter / 用线搜索参数更新控制
            u{k} = u{k} + alpha .* (du_ff) + du_fb + du_ft;
            if any(~isfinite(u{k}(:))),error('ddpack:numerics:NonfiniteTrajectory','Control update is nonfinite.');end
			% Apply control bounds / 应用控制边界
			u{k} = min(u_max, max(-u_max, u{k}));

            % Compute next state in trajectory with new control / 用新控制计算轨迹中的下一个状态
            x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, t_f) .* dt;

            % Return error if problem with trajectory / 如果轨迹有问题则返回错误
            if any(~isfinite(x_new{k+1}(:))) || any(~isfinite(u{k}(:)))
                error('ddpack:numerics:NonfiniteTrajectory','Candidate rollout is nonfinite.');
            end
		end

        %%% Update trajectory and time vector / 更新轨迹和时间向量
        % Update the current trajectory with new trajectory / 用新轨迹更新当前轨迹
        x = x_new;
		% Update time vector with new time horizon / 用新时间范围更新时间向量
		t = linspace(0.0, t_f, N);
		
% 		% Alternative: Adaptive terminal cost weight / 替代方案：自适应终端成本权重
% 		cost.Q_f = 1.50 * cost.Q_f;
        catch exception
            if isempty(lastFinite),rethrow(exception);end
            saved=lastFinite;
            sol=ddpack.numericalFailureSolution(saved,exception,numerical);
            sol=ddpack.finalizeTrajectoryCost(sol,cost,x_star);
            return;
        end
    end
    
    %% Assemble and return solution structure / 组装并返回解结构
    
%     fprintf("finished DDP, assembling results for post-processing...\n");
    fprintf("DDP iteration %d out of %d...\n", i, num_iter);
    
    % Assemble solution structure with all results / 用所有结果组装解结构
    sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, 0, i);
    sol.t=linspace(0,t_f,N);sol.dt=sol.t(2)-sol.t(1);sol.final_time=t_f;
    try
        sol=ddpack.finalizeTrajectoryCost(sol,cost,x_star);
    catch exception
        sol=ddpack.numericalFailureSolution(lastFinite,exception,numerical);
        sol=ddpack.finalizeTrajectoryCost(sol,cost,x_star);
        return;
    end
    sol.numerical_diagnostics=numerical;
    if converged
        sol.termination_reason='cost_change_below_tolerance';sol.convergence_status='converged_by_cost_change';
    else
        sol.termination_reason='maximum_iterations_reached';sol.convergence_status='maximum_iterations_reached_without_convergence';
    end
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
% Tf              : iteration history of time horizon / 时间范围的迭代历史
% Q_u, Q_uu, Q_ux : derivatives of state action value function / 状态动作值函数的导数
% error           : zero if no error, nonzero else / 无错误时为零，否则非零
% iter            : number of iterations completed / 完成的迭代次数
%
% Outputs / 输出参数
%
% sol : solution structure / 解结构
function sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, error, iter)

    % Solution structure / 解结构
    sol = struct;
    sol.error = error;        % Error flag / 错误标志
    sol.x = x;                % State trajectory / 状态轨迹
    sol.u = u;                % Control sequence / 控制序列
    sol.t = t;                % Time vector / 时间向量
    sol.dt = t(2) - t(1);     % Time step / 时间步长
    sol.J = J;                % Cost history / 成本历史
    sol.E = E;                % Energy history / 能量历史
    sol.Tf = Tf;              % Time horizon history / 时间范围历史
    sol.Q_u = Q_u;            % Q-function control gradient / Q函数控制梯度
    sol.Q_uu = Q_uu;          % Q-function control Hessian / Q函数控制海塞矩阵
    sol.Q_ux = Q_ux;          % Q-function cross derivative / Q函数交叉导数
	sol.iter = iter;          % Number of iterations / 迭代次数
    
    return
end
