%% Constrained Parameterized Differential Dynamic Programming Algorithm / 受约束参数化差分动态规划算法
% Obstacle avoidance (×), Waypoints (√) / 避障 (×), 航点 (√)

% Performs Differential Dynamic Programming Algorithm to produce a / 执行差分动态规划算法以产生
% locally optimal control sequence with waypoint constraints and free-time / 具有航点约束和自由时间
% optimization capabilities. / 优化能力的局部最优控制序列。
%
% This implementation extends the standard DDP algorithm to handle: / 此实现扩展了标准DDP算法以处理：
% - Multiple waypoints with time parameterization / 多个带时间参数化的航点
% - Free-time optimization for each trajectory segment / 每个轨迹段的自由时间优化
% - Constrained control inputs with clamping / 带约束的控制输入
% - Monte Carlo initialization options / 蒙特卡洛初始化选项
%
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
%
% Inputs / 输入参数：
% 
% montecarlo : flag for random initialization (1) or zero initialization (0) / 随机初始化标志 (1) 或零初始化 (0)
% freetime   : flag for free-time optimization (1) or fixed-time (0) / 自由时间优化标志 (1) 或固定时间 (0)
% x_0        : initial state vector / 初始状态向量
% x_wpts     : waypoint state vectors (2D array) / 航点状态向量 (2D数组)
% x_star     : target state vector / 目标状态向量
% t_f        : time horizon / 时间范围
% dt         : time step size / 时间步长
% dyn        : instance of Dynamics class / 动力学类实例
% cost       : instance of Cost class / 代价类实例
% u_max      : maximum magnitude of control, used for clamping / 控制的最大幅度，用于约束
% num_iter   : number of DDP iterations / DDP迭代次数
% alpha      : DDP learning parameter for line searching / DDP线搜索学习参数
%
% Outputs / 输出参数：
% 
% sol      : structure with solution components / 包含解决方案组件的结构体
% sol_init : structure with initial solution for comparison / 用于比较的初始解决方案结构体
%
function [sol,sol_init] = waypoint_pddp(montecarlo, freetime, x_0, x_wpts, x_star, t_f, dt, dyn, cost, u_max, num_iter, alpha)
    %% Allocate arrays for DDP / 为DDP分配数组

	% Calculate number of trajectory segments (waypoints + 1) / 计算轨迹段数 (航点数 + 1)
	segNum = size(x_wpts,2)+1;
    
    % Time discretization parameters / 时间离散化参数
    N = floor(t_f ./ dt);           % Total number of time steps / 总时间步数
    t_max = 15;                     % Maximum time per segment / 每段最大时间
    t_min = 5;                      % Minimum time per segment / 每段最小时间

    % Initialize normalized time vector for each segment / 初始化每个段的归一化时间向量
    t = linspace(0.0, 1, N/segNum);
    dt = t(2) - t(1);               % Recalculate dt based on normalized time / 基于归一化时间重新计算dt
    
    % Cost function history for convergence monitoring / 用于收敛监控的代价函数历史
    J = zeros(num_iter, 1);
    
    % Control energy history for analysis / 用于分析的控制能量历史
    E = zeros(num_iter, 1);
    
    % Time duration history for each segment / 每段时间持续历史
    Tf = zeros(num_iter, segNum);
    
    % State trajectory storage (current and updated) / 状态轨迹存储 (当前和更新)
    x = cell(N, 1);                 % Current trajectory / 当前轨迹
    x_new = cell(N, 1);             % Updated trajectory from forward pass / 来自前向传播的更新轨迹
    x{1} = x_0;                     % Initialize with starting state / 用起始状态初始化
    x_new{1} = x_0;
    
    % Control input trajectory / 控制输入轨迹
    u = cell(N, 1);
    
    % Value function and its derivatives (backward pass) / 值函数及其导数 (反向传播)
    V = zeros(N, 1);                % Value function V(x,t) / 值函数 V(x,t)
    V_x = cell(N, 1);               % Gradient w.r.t. state: dV/dx / 对状态的梯度: dV/dx
    V_t = cell(N, 1);               % Gradient w.r.t. time: dV/dt / 对时间的梯度: dV/dt
    V_xx = cell(N, 1);              % Hessian w.r.t. state: d²V/dx² / 对状态的Hessian: d²V/dx²
    V_xt = cell(N, 1);              % Mixed derivative: d²V/dxdt / 混合导数: d²V/dxdt
    V_tt = cell(N, 1);              % Hessian w.r.t. time: d²V/dt² / 对时间的Hessian: d²V/dt²
    
    % State-action value function Q(x,u,t) and its derivatives / 状态-动作值函数 Q(x,u,t) 及其导数
    Q_x = cell(N, 1);               % Gradient w.r.t. state: dQ/dx / 对状态的梯度: dQ/dx
    Q_u = cell(N, 1);               % Gradient w.r.t. control: dQ/du / 对控制的梯度: dQ/du
    Q_t = cell(N, 1);               % Gradient w.r.t. time: dQ/dt / 对时间的梯度: dQ/dt
    Q_xx = cell(N, 1);              % Hessian w.r.t. state: d²Q/dx² / 对状态的Hessian: d²Q/dx²
    Q_uu = cell(N, 1);              % Hessian w.r.t. control: d²Q/du² / 对控制的Hessian: d²Q/du²
    Q_tt = cell(N, 1);              % Hessian w.r.t. time: d²Q/dt² / 对时间的Hessian: d²Q/dt²
    Q_xu = cell(N, 1);              % Mixed derivative: d²Q/dxdu / 混合导数: d²Q/dxdu
    Q_xt = cell(N, 1);              % Mixed derivative: d²Q/dxdt / 混合导数: d²Q/dxdt
    Q_ux = cell(N, 1);              % Transpose of Q_xu / Q_xu的转置
    Q_ut = cell(N, 1);              % Mixed derivative: d²Q/dudt / 混合导数: d²Q/dudt
    Q_tu = cell(N, 1);              % Transpose of Q_ut / Q_ut的转置
	
    % Control policy gains (feedforward and feedback terms) / 控制策略增益 (前馈和反馈项)
    kt = cell(N, 1);                % Feedforward control gain / 前馈控制增益
    Kt = cell(N, 1);                % Feedback control gain matrix / 反馈控制增益矩阵
    Mt = cell(N, 1);                % Time feedback control gain / 时间反馈控制增益

	% State deviation for feedback control / 用于反馈控制的状态偏差
	dx = cell(N, 1);

    %% Initialize DDP with control sequence / 用控制序列初始化DDP
    % Generate initial control sequence based on montecarlo flag / 基于蒙特卡洛标志生成初始控制序列
	if montecarlo == 1
		fprintf("initializing with a random control sequence...\n")  % 用随机控制序列初始化...
	else
		fprintf("initializing with zero control sequence...\n")  % 用零控制序列初始化...
	end
    
    % Initialize control sequence for all time steps / 为所有时间步初始化控制序列
    for k = 1:N-1
		if montecarlo == 1
			% Random initialization: uniform distribution in [-u_max, u_max] / 随机初始化: 在 [-u_max, u_max] 中的均匀分布
			u{k} = 2 .* (u_max) .* rand(length(u_max), 1) - (u_max);
		else
			% Zero initialization / 零初始化
			u{k} = zeros(numel(u_max), 1);
		end
    end
    u{N} = zeros(numel(u_max), 1);  % Terminal control is always zero / 终端控制始终为零
    
%     fprintf("generating initial trajectory...\n")  % 生成初始轨迹...
    
    % Generate initial trajectory using forward integration / 使用前向积分生成初始轨迹
	for k = 1:N-1
		% Forward integration: x_{k+1} = x_k + F(x_k, u_k, t_f) * dt / 前向积分: x_{k+1} = x_k + F(x_k, u_k, t_f) * dt
		x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, t_f) .* dt;
	end
    x = x_new;  % Set current trajectory to the generated one / 将当前轨迹设置为生成的轨迹
    
    % Initialize state deviations (initially zero) / 初始化状态偏差 (初始为零)
	for k = 1:N
		dx{k} = (x_new{k} - x{k});
	end
	t_init = linspace(0.0, t_f, N);  % Initial time vector / 初始时间向量

	% Initialize time parameters for each segment / 初始化每个段的时间参数
	theta = t_f/segNum * ones(segNum,1);  % Equal time allocation per segment / 每段相等的时间分配
	dtheta = zeros(segNum,1);             % Time updates (initially zero) / 时间更新 (初始为零)

	% Store initial solution for comparison / 存储初始解决方案用于比较
	Tf_init(1,:) = theta';
	sol_init = assemble_solution(x, u, t_init, J, E, Tf_init, Q_u, Q_uu, Q_ux, 0, 1);

	% Algorithm parameters / 算法参数
	sigma = 50.0;        % Waypoint constraint penalty weight / 航点约束惩罚权重
	LM_rhoQ = 1e-6;      % Levenberg-Marquardt regularization for Q_uu / Q_uu的Levenberg-Marquardt正则化
	LM_rhoV = 1e-6;      % Levenberg-Marquardt regularization for V_tt / V_tt的Levenberg-Marquardt正则化
    
    %% Perform main DDP iterations on the trajectory and input sequence / 对轨迹和输入序列执行主要DDP迭代
    
%     fprintf("beginning DDP...\n")  % 开始DDP...
    
    converged=false;
    lastFinite=[];
    numerical=struct('linear_solves',0,'regularized_solves',0,'maximum_damping',0,'backward_retries',0);
    for i = 1:num_iter
        saved=struct('x',{x},'u',{u},'t',linspace(0,sum(theta),N),'J',J(1:max(1,i-1)),'iter',i,'final_time',sum(theta));
        try
%         fprintf("DDP iteration %d out of %d...\n", i, num_iter);  % DDP迭代 %d / %d...
        
        % Compute total cost for current iteration / 计算当前迭代的总代价
        J(i) = cost.phi(x{N}, x_star);  % Terminal cost / 终端代价
		j = 1;  % Current segment index / 当前段索引
		for k = 1:N-1
			if k >= (j-1)*N/segNum  % Check if we're in the current segment / 检查是否在当前段
				% Add running cost for current time step / 为当前时间步添加运行代价
				J(i) = J(i) + cost.L(x{k}, x_star, u{k}, theta(j), dt);
				% Compute control energy usage (quadratic in control) / 计算控制能量使用 (控制的二次型)
				E(i) = E(i) + theta(j)*0.5 .* u{k}.' * u{k} .* dt;
				if k == j*N/segNum  % Move to next segment / 移动到下一段
					j = j+1;
				end
			end
		end
		Tf(i,:) = theta';  % Store current time allocation / 存储当前时间分配

        % Commit a nominal only after its complete cost and trajectory pass.
        saved.J=J(1:i);
        if ~isfinite(J(i)),error('ddpack:numerics:NonfiniteTrajectory','Nominal objective is nonfinite.');end
        if ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),x)) || ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),u))
            error('ddpack:numerics:NonfiniteTrajectory','Nominal trajectory is nonfinite.');
        end
        lastFinite=saved;

        % Check for convergence (cost change below threshold) / 检查收敛性 (代价变化低于阈值)
        if i > 1 && abs(J(i) - J(i-1)) < 1e-5
            fprintf("Convergence at %d-th iter!!! ...\n", i)  % 在第 %d 次迭代收敛!!!
            u{N} = u{N-1};  % Set terminal control to previous step / 将终端控制设置为前一步
            converged=true;break;
        end
        
        % Compute terminal value function and derivatives (boundary conditions) / 计算终端值函数及其导数 (边界条件)
        V(N) = cost.phi(x{N}, x_star);        % Terminal value function / 终端值函数
        V_x{N} = cost.phi_x(x{N}, x_star);    % Terminal value gradient w.r.t. state / 终端值对状态的梯度
        V_t{N} = cost.phi_t(x{N}, x_star);    % Terminal value gradient w.r.t. time / 终端值对时间的梯度
        V_xx{N} = cost.phi_xx(x{N}, x_star);  % Terminal value Hessian w.r.t. state / 终端值对状态的Hessian
        V_xt{N} = cost.phi_xt(x{N}, x_star);  % Terminal value mixed derivative / 终端值混合导数
        V_tt{N} = cost.phi_tt(x{N}, x_star);  % Terminal value Hessian w.r.t. time / 终端值对时间的Hessian
		
% 		if i>1
% 		end
        
        % Perform backwards pass (compute value function and control gains) / 执行反向传播 (计算值函数和控制增益)
		j = segNum;              % Start from last segment / 从最后一段开始
		BackwarkPass = 0;        % Flag to indicate if backward pass completed successfully / 标志指示反向传播是否成功完成
		backwardAttempts=0;
        while ~BackwarkPass
            backwardAttempts=backwardAttempts+1;
            if backwardAttempts>80
                error('ddpack:numerics:BackwardRetriesExhausted','Backward pass exceeded 80 attempts.');
            end
            numerical.backward_retries=numerical.backward_retries+double(backwardAttempts>1);
            j=segNum;
            V_x{N}=cost.phi_x(x{N},x_star);V_xx{N}=cost.phi_xx(x{N},x_star);
            V_t{N}=cost.phi_t(x{N},x_star);V_tt{N}=cost.phi_tt(x{N},x_star);V_xt{N}=cost.phi_xt(x{N},x_star);
			% Iterate backwards through time steps / 反向遍历时间步
			for k = N-1:-1:1
				% Compute state-action value function Q(x,u,t) derivatives / 计算状态-动作值函数 Q(x,u,t) 的导数
				% Q_x: gradient w.r.t. state (running cost + dynamics contribution) / Q_x: 对状态的梯度 (运行代价 + 动力学贡献)
				Q_x{k} = cost.L_x(x{k}, x_star, u{k}, theta(j), dt) + dyn.Phi(x{k}, u{k}, theta(j), dt)' * V_x{k+1};
				% Q_u: gradient w.r.t. control (running cost + dynamics contribution) / Q_u: 对控制的梯度 (运行代价 + 动力学贡献)
				Q_u{k} = cost.L_u(x{k}, u{k}, theta(j), dt) + dyn.beta(x{k}, u{k}, theta(j), dt)' * V_x{k+1};
				% Q_t: gradient w.r.t. time (running cost + value function + dynamics contribution) / Q_t: 对时间的梯度 (运行代价 + 值函数 + 动力学贡献)
				Q_t{k} = cost.L_t(x{k}, x_star, u{k}, theta(j), dt) + V_t{k+1} + dyn.theta(x{k}, u{k}, theta(j), dt).' * V_x{k+1};
				% Q_xx: Hessian w.r.t. state (with Levenberg-Marquardt regularization) Q_xx: 对状态的Hessian (带Levenberg-Marquardt正则化)
				Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, theta(j), dt) + dyn.Phi(x{k}, u{k}, theta(j), dt)' * (V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.Phi(x{k}, u{k}, theta(j), dt);
				% Q_uu: Hessian w.r.t. control (with regularization) Q_uu: 对控制的Hessian (带正则化)
				Q_uu{k} = cost.L_uu(x{k}, u{k}, theta(j), dt) + dyn.beta(x{k}, u{k}, theta(j), dt)' * (V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.beta(x{k}, u{k}, theta(j), dt);
				% Q_xu: Mixed derivative w.r.t. state and control Q_xu: 对状态和控制的混合导数
				Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + dyn.Phi(x{k}, u{k}, theta(j), dt)' * (V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.beta(x{k}, u{k}, theta(j), dt);
				Q_ux{k} = Q_xu{k}';  % Symmetric property 对称性质
				% Q_xt: Mixed derivative w.r.t. state and time Q_xt: 对状态和时间的混合导数
				Q_xt{k} = cost.L_xt(x{k}, x_star, u{k}, theta(j), dt) + dyn.Phi(x{k}, u{k}, theta(j), dt)' * V_xt{k+1} ...
									 		 			+ dyn.Phi(x{k}, u{k}, theta(j), dt)' * (V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.theta(x{k}, u{k}, theta(j), dt);
				Q_tx{k} = Q_xt{k}';  % Symmetric property 对称性质
				% Q_ut: Mixed derivative w.r.t. control and time Q_ut: 对控制和时间的混合导数
				Q_ut{k} = cost.L_ut(x{k}, u{k}, theta(j), dt) + dyn.beta(x{k}, u{k}, theta(j), dt)' * V_xt{k+1} ...
									 		 			+ dyn.beta(x{k}, u{k}, theta(j), dt)' * (V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.theta(x{k}, u{k}, theta(j), dt);
				Q_tu{k} = Q_ut{k}';  % Symmetric property 对称性质
				% Q_tt: Hessian w.r.t. time Q_tt: 对时间的Hessian
				Q_tt{k} = cost.L_tt(x{k}, u{k}, theta(j), dt) + V_tt{k+1} + 2*dyn.theta(x{k}, u{k}, theta(j), dt)'*V_xt{k+1}...
											 			+ dyn.theta(x{k}, u{k}, theta(j), dt)'*(V_xx{k+1}+LM_rhoQ*eye(length(x{k}))) * dyn.theta(x{k}, u{k}, theta(j), dt);
	
				% Add waypoint constraint penalty (only for position coordinates) 添加航点约束惩罚 (仅针对位置坐标)
				if k == (j-1)*N/segNum
					% Position deviation from waypoint (only first 2 coordinates) 与航点的位置偏差 (仅前2个坐标)
					Dx = [(x{k}(1:2, 1)- x_wpts(:,j-1)); zeros(numel(x{k})-2, 1)].*theta(j)*dt;
					Q_x{k} = Q_x{k} + sigma*Dx;  % Add penalty gradient 添加惩罚梯度
					Q_xx{k} = Q_xx{k} + sigma*eye(length(x{k})) * theta(j)*dt;  % Add penalty Hessian 添加惩罚Hessian
				end
				
				% Check positive definiteness of Q_uu using Cholesky decomposition 使用Cholesky分解检查Q_uu的正定性
                if any(~isfinite(Q_uu{k}(:)))
                    error('ddpack:numerics:NonfiniteLinearSystem','Nonfinite waypoint control Hessian.');
                end
                [~,cholFlag]=chol(Q_uu{k});
                if cholFlag~=0
					fprintf("Q_uu is not definite in %d-iter %d-instant.\n", i, k);
					% Increase Levenberg-Marquardt regularization 增加Levenberg-Marquardt正则化
					LM_rhoQ = LM_rhoQ * 2.0;
					if ~isfinite(LM_rhoQ)
					    error('ddpack:numerics:BackwardRetriesExhausted','Nonfinite backward damping.');
					end
					break;  % Restart backward pass with increased regularization 用增加的正则化重新开始反向传播
    			end
					
				% Compute optimal control gains (feedforward and feedback terms) 计算最优控制增益 (前馈和反馈项)
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},-[Q_u{k},Q_ux{k},Q_ut{k}],numerical);
            kt{k}=gains(:,1);Kt{k}=gains(:,2:1+numel(x_0));Mt{k}=gains(:,end);

	%     		% Alternative formulation (commented out) 替代公式 (已注释)
	%     		V_x{k} = Q_x{k} - Q_xu{k} * (Q_uu{k} \ Q_u{k});
	%     		V_t{k} = Q_t{k} - Q_tu{k} * (Q_uu{k} \ Q_u{k});
	%     		V_xx{k} = Q_xx{k} - Q_xu{k} * (Q_uu{k} \ Q_ux{k});
	%     		V_xt{k} = Q_xt{k} - Q_xu{k} * (Q_uu{k} \ Q_ut{k});
	%     		V_tx{k} = V_xt{k}';
	%     		V_tt{k} = Q_tt{k} - Q_tu{k} * (Q_uu{k} \ Q_ut{k});
				
				% Compute value function derivatives using optimal control gains 使用最优控制增益计算值函数导数
				V_x{k} = Q_x{k} + Kt{k}'*Q_u{k} + Q_xu{k}*kt{k} + Kt{k}'*Q_uu{k}*kt{k};
				V_t{k} = Q_t{k} + Mt{k}'*Q_u{k} + Q_tu{k}*kt{k} + Mt{k}'*Q_uu{k}*kt{k};
				V_xx{k} = Q_xx{k} + Q_xu{k}*Kt{k} + Kt{k}'*Q_ux{k} + Kt{k}'*Q_uu{k}*Kt{k};
				V_xt{k} = Q_xt{k} + Q_xu{k}*Mt{k} + Kt{k}'*Q_ut{k} + Kt{k}'*Q_uu{k}*Mt{k};
				V_tt{k} = Q_tt{k} + Q_tu{k}*Mt{k} + Mt{k}'*Q_ut{k} + Mt{k}'*Q_uu{k}*Mt{k};

				% Add Levenberg-Marquardt regularization to V_tt 向V_tt添加Levenberg-Marquardt正则化
				V_tt{k} = (V_tt{k}+LM_rhoV*eye(length(V_tt{k})));

				% Check positive definiteness of V_tt using Cholesky decomposition 使用Cholesky分解检查V_tt的正定性
                if any(~isfinite(V_tt{k}(:)))
                    error('ddpack:numerics:NonfiniteLinearSystem','Nonfinite waypoint time Hessian.');
                end
                [~,cholFlag]=chol(V_tt{k});
                if cholFlag~=0
					fprintf("V_tt is not definite in %d-iter %d-instant.\n", i, k);
					% Increase Levenberg-Marquardt regularization 增加Levenberg-Marquardt正则化
					LM_rhoV = LM_rhoV * 2.0;
					if ~isfinite(LM_rhoV)
					    error('ddpack:numerics:BackwardRetriesExhausted','Nonfinite backward damping.');
					end
					break;  % Restart backward pass with increased regularization 用增加的正则化重新开始反向传播
    			end
	
				% Update time parameters at segment boundaries 在段边界更新时间参数
				if k == (j-1)*N/segNum || k == 1
					if freetime == 1
						% Free-time optimization: update segment duration 自由时间优化: 更新段持续时间
						dtheta(j) = ddpack.checkedTimeSolve(V_tt{k},-(V_t{k} + V_xt{k}' * dx{k}));
					elseif freetime == 0
						% Fixed-time: no time updates 固定时间: 无时间更新
						dtheta(j) = 0;
					end

					j = j-1;  % Move to previous segment 移动到前一段
					if k == 1  % Completed backward pass 完成反向传播
						% Reset regularization parameters for next iteration 为下一次迭代重置正则化参数
						LM_rhoQ = 1e-5;
						LM_rhoV = 1e-5;
						BackwarkPass = 1;  % Mark backward pass as completed 标记反向传播为已完成
					end
				end
			end
		end
% 		if i > 20
% 			sigma = 1.2*sigma;
% 		end

		% Update time parameters with constraints 用约束更新时间参数
		theta = theta + dtheta;
        % Apply time bounds to each segment 对每段应用时间边界
        for m = 1:numel(theta)
            theta(m) = min(t_max, max(t_min, theta(m)));
        end

        % Forward pass: Update control sequence and generate new trajectory 前向传播: 更新控制序列并生成新轨迹
		j = 1;  % Current segment index 当前段索引
		for k = 1:N-1
			if k >= (j-1)*N/segNum  % Check if we're in the current segment 检查是否在当前段
        		% Compute control update components 计算控制更新组件
        		du_ff = kt{k};                              % Feedforward term 前馈项
        		du_fb = Kt{k} * (x_new{k} - x{k});         % State feedback term 状态反馈项
        		du_ft = Mt{k} * dtheta(j);                 % Time feedback term 时间反馈项

				dx{k} = (x_new{k} - x{k});  % State deviation for feedback 用于反馈的状态偏差

	%                 % Alternative control clamping (commented out) 替代控制约束 (已注释)
	%             for m = 1:numel(u_max)
	%                 du_ff(m) = min(u_max(m), max(-u_max(m), ...
	%                                      du_ff(m) + u{k}(m))) - u{k}(m);
	%             end

        		% Update control with all terms 用所有项更新控制
% 				if mod(i,2) == 1
					u{k} = u{k} + alpha .* (du_ff) + du_fb + du_ft;
            if any(~isfinite(u{k}(:))),error('ddpack:numerics:NonfiniteTrajectory','Control update is nonfinite.');end
% 				elseif mod(i,2) == 0
% 					u{k} = u{k} + du_fb + du_ft;
% 				end
				% Apply control constraints (clamping) 应用控制约束 (约束)
				u{k} = min(u_max, max(-u_max, u{k}));

        		% Forward integration: compute next state with updated control 前向积分: 用更新的控制计算下一状态
        		x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, theta(j)) .* dt;
	
        		% Check for numerical issues in trajectory 检查轨迹中的数值问题
				if any(~isfinite(x_new{k+1}(:))) || any(~isfinite(u{k}(:)))
				    error('ddpack:numerics:NonfiniteTrajectory','Candidate rollout is nonfinite.');
				end
				if k == j*N/segNum  % Move to next segment 移动到下一段
					j = j+1;
				end
			end
		end

        % Update trajectory and time vector after forward pass 前向传播后更新轨迹和时间向量
        x = x_new;  % Set current trajectory to the updated one 将当前轨迹设置为更新的轨迹
        
        % Compute total time and update time vector 计算总时间并更新时间向量
		t_total = 0;
		for j = 1:segNum
			t_total = t_total + theta(j);
		end
		t = linspace(0.0, t_total, N);  % Update time vector with new total time 用新的总时间更新时间向量
        catch exception
            if isempty(lastFinite),rethrow(exception);end
            saved=lastFinite;
            sol=ddpack.numericalFailureSolution(saved,exception,numerical);
            sol=ddpack.finalizeTrajectoryCost(sol,cost,x_star);
            return;
        end
    end
    
    %% Assemble and return solution structure 组装并返回解决方案结构
    
    fprintf("finished DDP, assembling results for post-processing...\n");
    
    % Assemble final solution with all computed data 用所有计算数据组装最终解决方案
    sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, 0, i);
    sol.t=linspace(0,sum(theta),N);sol.dt=sol.t(2)-sol.t(1);sol.final_time=sum(theta);
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

%% Helper Functions for DDP Algorithm DDP算法的辅助函数

% Assembles and returns solution structure containing all optimization results 组装并返回包含所有优化结果的解决方案结构
%
% This function packages all the computed data from the DDP algorithm into 此函数将DDP算法中的所有计算数据打包成
% a structured format for easy access and post-processing. 结构化格式以便于访问和后处理。
%
% Inputs 输入参数
%
% x               : locally optimal state trajectory (cell array) 局部最优状态轨迹 (单元数组)
% u               : locally optimal control sequence (cell array) 局部最优控制序列 (单元数组)
% t               : discretized time stamps (vector) 离散化时间戳 (向量)
% J               : iteration history of cost function (vector) 代价函数的迭代历史 (向量)
% E               : iteration history of control energy (vector) 控制能量的迭代历史 (向量)
% Tf              : time duration history for each segment (matrix) 每段时间持续历史 (矩阵)
% Q_u, Q_uu, Q_ux : derivatives of state action value function (cell arrays) 状态动作值函数的导数 (单元数组)
% error           : error flag (0 if no error, 1 if error occurred) 错误标志 (无错误为0，发生错误为1)
% iter            : number of iterations completed (integer) 完成的迭代次数 (整数)
%
% Outputs 输出参数
%
% sol : solution structure containing all optimization results 包含所有优化结果的解决方案结构
function sol = assemble_solution(x, u, t, J, E, Tf, Q_u, Q_uu, Q_ux, error, iter)

    % Create solution structure with all optimization results 创建包含所有优化结果的解决方案结构
    sol = struct;
    sol.error = error;        % Error flag indicating success/failure 指示成功/失败的错误标志
    sol.x = x;                % Optimal state trajectory 最优状态轨迹
    sol.u = u;                % Optimal control sequence 最优控制序列
    sol.t = t;                % Time vector 时间向量
    sol.dt = t(2) - t(1);     % Time step size 时间步长
    sol.J = J;                % Cost function history 代价函数历史
    sol.E = E;                % Control energy history 控制能量历史
    sol.Tf = Tf;              % Time duration history for each segment 每段时间持续历史
    sol.Q_u = Q_u;            % Q-function gradient w.r.t. control Q函数对控制的梯度
    sol.Q_uu = Q_uu;          % Q-function Hessian w.r.t. control Q函数对控制的Hessian
    sol.Q_ux = Q_ux;          % Q-function mixed derivative w.r.t. control and state Q函数对控制和状态的混合导数
    sol.iter = iter;          % Number of iterations completed 完成的迭代次数
    
    return
end
