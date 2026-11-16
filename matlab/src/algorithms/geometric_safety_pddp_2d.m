%% Geometric safety correction Embedded Terminal-defect Parameterized Differential Dynamic Programming Algorithm / 控制屏障函数嵌入多重射击参数化差分动态规划算法
% Obstacle avoidance (√), Waypoints (√) / 避障 (√), 航点 (√)

% Performs geometry-aware safety-corrected PDDP Algorithm to produce a locally optimal control sequence / 执行geometry-aware safety-corrected PDDP算法以产生局部最优控制序列
% with obstacle avoidance capabilities using Geometric safety corrections. / 使用控制屏障函数具有避障能力。
%
% This implementation extends the standard PDDP algorithm to handle: / 此实现扩展了标准PDDP算法以处理：
% - Geometric safety correction (geometric-safety) constraints for obstacle avoidance / 用于避障的控制屏障函数(geometric-safety)约束
% - Multiple obstacles with different positions and radii / 具有不同位置和半径的多个障碍物
% - Free-time optimization for trajectory duration / 轨迹持续时间的自由时间优化
% - Constrained control inputs with clamping / 带约束的控制输入
% - Proportional guidance initialization / 比例制导初始化
%
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
%
% Inputs / 输入参数：
% 
% x_0            : initial state vector / 初始状态向量
% x_star         : target state vector / 目标状态向量
% t_f            : time horizon / 时间范围
% dt             : time step size / 时间步长
% dyn            : instance of Dynamics class / 动力学类实例
% cost           : instance of Cost class / 代价类实例
% u_max          : maximum magnitude of control, used for clamping / 控制的最大幅度，用于约束
% num_iter       : number of geometry-aware safety-corrected PDDP iterations / geometry-aware safety-corrected PDDP迭代次数
% epsilon        : geometry-aware safety-corrected PDDP learning parameter for line searching / geometry-aware safety-corrected PDDP线搜索学习参数
% rho            : defect weight update parameter / 缺陷权重更新参数
% update_epsilon : epsilon update factor / epsilon更新因子
% obs_position   : obstacle positions (2D array) / 障碍物位置 (2D数组)
% obs_radius     : obstacle radii (vector) / 障碍物半径 (向量)
% gamma_h        : geometric-safety safety parameter / geometric-safety安全参数
% single_obs     : flag for single obstacle (1) or multiple obstacles (0) / 单障碍物标志 (1) 或多障碍物 (0)
% num_obs        : number of obstacles / 障碍物数量
%
% Outputs / 输出参数：
% 
% sol : structure with solution components / 包含解决方案组件的结构体
%
function sol = geometric_safety_pddp_2d(x_0, x_star, t_f, dt, dyn, cost, u_max, num_iter, epsilon, rho, update_epsilon, obs_position, obs_radius, gamma_h, single_obs, num_obs, minimum_time, maximum_time)
%% Allocate arrays for geometry-aware safety-corrected PDDP 为geometry-aware safety-corrected PDDP分配数组

    % Time discretization parameters / 时间离散化参数
    N = floor(t_f ./ dt);           % Total number of time steps / 总时间步数
    if nargin < 17 || isempty(minimum_time)
        minimum_time = max(t_f-20, 5);
    end
    if nargin < 18 || isempty(maximum_time)
        maximum_time = t_f + 7;
    end
    t_min = minimum_time;           % Scenario-defined minimum maneuver time
    t_max = maximum_time;           % Scenario-defined maximum maneuver time

    % Normalized time vector / 归一化时间向量
    t = linspace(0.0, 1, N);
    dt = t(2) - t(1);               % Recalculate dt based on normalized time / 基于归一化时间重新计算dt
    
    % Cost function history for convergence monitoring / 用于收敛监控的代价函数历史
    validateattributes(rho,{'numeric'},{'scalar','real','finite','>',0,'<',1});
    lineSearchTrials=zeros(0,18);costHistory=[];meritHistory=[];
    meritWeight=NaN;lineSearchStalled=false;acceptedSteps=0;
    J = zeros(num_iter, 1);
    M = zeros(num_iter, 1);         % Merit function history / 价值函数历史
    % merit = cell(N, 1);           % Merit function at each time step (commented out) / 每个时间步的价值函数 (已注释)
    
    % Control energy history for analysis / 用于分析的控制能量历史
    E = zeros(num_iter, 1);

    % geometric safety value history for each obstacle / 每个障碍物的geometric-safety约束历史
    h_x_history = zeros(N, num_obs);
    
    % Time parameter history / 时间参数历史
    Theta = zeros(num_iter, 1);

    % State trajectory storage (current and updated) / 状态轨迹存储 (当前和更新)
    x = cell(N, 1);                 % Current trajectory / 当前轨迹
    x_new = cell(N, 1);             % Updated trajectory from forward pass / 来自前向传播的更新轨迹
    x{1} = x_0;                     % Initialize with starting state / 用起始状态初始化
    x_new{1} = x_0;

    % xlocal{1} = x{1};             % Local state (commented out) / 局部状态 (已注释)
    
    % Control input trajectory / 控制输入轨迹
    u = cell(N, 1);
    
    % Distance to obstacles history / 到障碍物距离历史
    dist_to_obs_history = zeros(N, 1);
    
    % Value function and its derivatives (backward pass) / 值函数及其导数 (反向传播)
    V = zeros(N, 1);                % Value function V(x,t) / 值函数 V(x,t)
    V_x = cell(N, 1);               % Gradient w.r.t. state: dV/dx / 对状态的梯度: dV/dx
    V_t = cell(N, 1);               % Gradient w.r.t. time: dV/dt / 对时间的梯度: dV/dt
    % V_xx1 = cell(N, 1);          % Regularized Hessian (commented out) / 正则化Hessian (已注释)
    V_xx = cell(N, 1);              % Hessian w.r.t. state: d²V/dx² / 对状态的Hessian: d²V/dx²
    V_xt = cell(N, 1);              % Mixed derivative: d²V/dxdt / 混合导数: d²V/dxdt
    V_tt = cell(N, 1);              % Hessian w.r.t. time: d²V/dt² / 对时间的Hessian: d²V/dt²
    % V_tt1 = cell(N, 1);          % Regularized time Hessian (commented out) / 正则化时间Hessian (已注释)

    % State-action value function Q(x,u,t) and its derivatives / 状态-动作值函数 Q(x,u,t) 及其导数
    Q_x = cell(N, 1);               % Gradient w.r.t. state: dQ/dx / 对状态的梯度: dQ/dx
    Q_u = cell(N, 1);               % Gradient w.r.t. control: dQ/du / 对控制的梯度: dQ/du
    Q_t = cell(N, 1);               % Gradient w.r.t. time: dQ/dt / 对时间的梯度: dQ/dt
    Q_xx = cell(N, 1);              % Hessian w.r.t. state: d²Q/dx² / 对状态的Hessian: d²Q/dx²
    Q_xu = cell(N, 1);              % Mixed derivative: d²Q/dxdu / 混合导数: d²Q/dxdu
    Q_ux = cell(N, 1);              % Transpose of Q_xu / Q_xu的转置
    Q_xt = cell(N, 1);              % Mixed derivative: d²Q/dxdt / 混合导数: d²Q/dxdt
    Q_tx = cell(N, 1);              % Transpose of Q_xt / Q_xt的转置
    Q_uu = cell(N, 1);              % Hessian w.r.t. control: d²Q/du² / 对控制的Hessian: d²Q/du²
    Q_ut = cell(N, 1);              % Mixed derivative: d²Q/dudt / 混合导数: d²Q/dudt
    Q_tu = cell(N, 1);              % Transpose of Q_ut / Q_ut的转置
    Q_tt = cell(N, 1);              % Hessian w.r.t. time: d²Q/dt² / 对时间的Hessian: d²Q/dt²

    % Control policy gains (feedforward and feedback terms) / 控制策略增益 (前馈和反馈项)
    k_t = cell(N, 1);               % Feedforward control gain / 前馈控制增益
    K_t = cell(N, 1);               % State feedback gain matrix / 状态反馈增益矩阵
    M_t = cell(N, 1);               % Time feedback control gain / 时间反馈控制增益
    
    % log_d = [];                   % Log of defect vectors (commented out) / 缺陷向量日志 (已注释)
    d = zeros(numel(x),1);          % Defect vector for terminal-defect correction / 多重射击的缺陷向量
    % act_merit_change = zeros(num_iter, 1);  % Actual merit change (commented out) / 实际价值变化 (已注释)
    % exp_cost_change = zeros(num_iter, 1);   % Expected cost change (commented out) / 期望代价变化 (已注释)
    % exp_merit_change = zeros(num_iter, 1);  % Expected merit change (commented out) / 期望价值变化 (已注释)

    %% Initialize geometry-aware safety-corrected PDDP with proportional guidance / 用比例制导初始化geometry-aware safety-corrected PDDP

    % fprintf("initializing random control sequence...\n")  % 用随机控制序列初始化...
    % 
    % % Generate random control sequence / 生成随机控制序列
    % for k = 1:N-1
    %     u{k} = zeros(numel(u_max), 1);
    % end
    % u{N} = zeros(numel(u_max), 1);
    % 
    % fprintf("generating initial trajectory...\n")  % 生成初始轨迹...
    % 
    % % Generate initial trajectory using random control sequence / 使用随机控制序列生成初始轨迹
    % for k = 1:N-1
    %     x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, t_f) .* dt;
    % end
    % x = x_new;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Use proportional guidance for initialization / 使用比例制导进行初始化
    [path, control] = Proportional_guidance(x_0, x_star, t_f, dt);
    u = control;                    % Set control sequence / 设置控制序列
    x = path;                       % Set initial trajectory / 设置初始轨迹
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Perform main geometry-aware safety-corrected PDDP iterations on the trajectory, input sequence and parameter / 对轨迹、输入序列和参数执行主要geometry-aware safety-corrected PDDP迭代
    
    % fprintf("beginning geometry-aware safety-corrected PDDP...\n")  % 开始geometry-aware safety-corrected PDDP...
    
    x=x(1:N);u=u(1:N);t_f=min(t_max,max(t_min,t_f));
    x{1}=x_0;
    for seedIndex=1:N-1
        u{seedIndex}=min(u_max,max(-u_max,u{seedIndex}));
        x{seedIndex+1}=x{seedIndex}+dt*dyn.F(x{seedIndex},u{seedIndex},t_f);
    end
    u{N}=u{N-1};x_new=x;
    converged = false;
    completedIterations = 0;
    lastFinite=[];
    numerical=struct('linear_solves',0,'regularized_solves',0,'maximum_damping',0,'backward_retries',0);
    for i = 1:num_iter
        saved=struct('x',{x},'u',{u},'t',t,'J',J(1:max(1,i-1)),'iter',i,'final_time',t_f);
        try
        completedIterations = i;
        % fprintf("geometry-aware safety-corrected PDDP iteration %d out of %d...\n", i, num_iter);  % geometry-aware safety-corrected PDDP迭代 %d / %d...
        
        % Compute total cost for current iteration / 计算当前迭代的总代价
        J(i) = cost.phi(x{N}, x_star);  % Terminal cost / 终端代价
        for k = 1:N-1
            J(i) = J(i) + cost.L(x{k}, x_star, u{k}, t_f, dt);  % Running cost / 运行代价
        end

        saved.J=J(1:i);
        if ~isfinite(J(i)),error('ddpack:numerics:NonfiniteTrajectory','Nominal objective is nonfinite.');end
        if ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),x)) || ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),u))
            error('ddpack:numerics:NonfiniteTrajectory','Nominal trajectory is nonfinite.');
        end
        lastFinite=saved;
        % Compute control energy usage (quadratic in control) / 计算控制能量使用 (控制的二次型)
        for k = 1:N-1
            E(i) = E(i) + t_f * 0.5 .* u{k}.' * u{k} .* dt;
        end
        Theta(i) = t_f;             % Store current time parameter / 存储当前时间参数

        % Check for convergence (cost change below threshold) / 检查收敛性 (代价变化低于阈值)
        if i > 1 && abs(J(i) - J(i-1)) < 1e-5
            % fprintf("PDDP iteration %d out of %d...\n", i, num_iter);  % PDDP迭代 %d / %d...
            % disp('convergence');  % 显示收敛
            fprintf("Convergence at iteration %d out of %d...\n", i, num_iter);  % 在第 %d 次迭代收敛，共 %d 次...
            u{N} = u{N-1};          % Set terminal control to previous step / 将终端控制设置为前一步
            converged = true;
            break;
        end

        % Compute terminal value function and derivatives (boundary conditions) / 计算终端值函数及其导数 (边界条件)
        V(N) = cost.phi(x{N}, x_star);        % Terminal value function / 终端值函数
        V_x{N} = cost.phi_x(x{N}, x_star);    % Terminal value gradient w.r.t. state / 终端值对状态的梯度
        V_xx{N} = cost.phi_xx(x{N}, x_star);  % Terminal value Hessian w.r.t. state / 终端值对状态的Hessian
        % V_t{N} = zeros(numel(s));           % Terminal value gradient w.r.t. time (commented out) / 终端值对时间的梯度 (已注释)
        V_t{N} = cost.phi_t(x{N}, x_star);    % Terminal value gradient w.r.t. time / 终端值对时间的梯度
        % V_tt{N} = zeros(numel(s),numel(s)); % Terminal value Hessian w.r.t. time (commented out) / 终端值对时间的Hessian (已注释)
        V_tt{N} = cost.phi_tt(x{N}, x_star);  % Terminal value Hessian w.r.t. time / 终端值对时间的Hessian
        % V_xt{N} = zeros(numel(x_star), numel(s)); % Terminal value mixed derivative (commented out) / 终端值混合导数 (已注释)
        V_xt{N} = cost.phi_xt(x{N}, x_star);  % Terminal value mixed derivative / 终端值混合导数

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Compute defect vector for terminal-defect correction / 计算多重射击的缺陷向量
        d = x{N}- x_star;           % State deviation at terminal time / 终端时间的状态偏差
        % d = zeros(3,1);           % Zero defect (commented out) / 零缺陷 (已注释)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Perform backwards pass (compute value function and control gains) 执行反向传播 (计算值函数和控制增益)
        for k = N-1:-1:1
           
            % Regularized derivatives (commented out) 正则化导数 (已注释)
            % V_xx1{k+1} = V_xx{k+1} + 0.01 * eye(size(V_xx{k+1}));
            % V_tt1{k+1} = V_tt{k+1} + 0.01 * eye(size(V_tt{k+1}));
            % V_xx1{k+1} = V_xx{k+1};
            % V_ss1{k+1} = V_ss{k+1};

            % Compute state-action value function Q(x,u,t) derivatives 计算状态-动作值函数 Q(x,u,t) 的导数
            if k == N-1
                % Special case for last time step with defect vector 带缺陷向量的最后时间步特殊情况
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                %3*1
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                %1*1
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1}...
                    + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d + V_xt{k+1}' * d;
            %1*1
            else
                % Standard case for other time steps 其他时间步的标准情况
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                %3*1
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                %1*1
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + ...
                    dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1};
            %1*1
            end

            % Compute second-order derivatives 计算二阶导数
            Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, t_f, dt);
            %3*3
            Q_uu{k} = cost.L_uu(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            %1*1
            Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            %3*1
            Q_ux{k} = Q_xu{k}';     % Symmetric property 对称性质
            %1*3
            Q_xt{k} = cost.L_xt(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %3*1
            Q_tx{k} = Q_xt{k}';     % Symmetric property 对称性质
            %1*3
            Q_ut{k} = cost.L_ut(x{k}, u{k}, t_f, dt) + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %1*1
            Q_tu{k} = Q_ut{k}';     % Symmetric property 对称性质
            %1*1
            Q_tt{k} = cost.L_tt(x{k}, u{k}, t_f, dt) + V_tt{k+1} + ...
                2 * dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %1*1

            % Compute optimal control gains (feedforward and feedback terms) 计算最优控制增益 (前馈和反馈项)
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},-[Q_u{k},Q_ux{k},Q_ut{k}],numerical);
            k_t{k}=gains(:,1);K_t{k}=gains(:,2:1+numel(x_0));M_t{k}=gains(:,end);

            % Compute value function derivatives using optimal control gains 使用最优控制增益计算值函数导数
            V_x{k} = Q_x{k} + K_t{k}' * Q_u{k} + Q_xu{k} * k_t{k} + K_t{k}' * Q_uu{k} * k_t{k};
            %3*1
            V_t{k} = Q_t{k} + M_t{k}' * Q_u{k} + Q_tu{k} * k_t{k} + M_t{k}' * Q_uu{k} * k_t{k};
            %1*1
            V_xx{k} = Q_xx{k} + Q_xu{k} * K_t{k} + K_t{k}' * Q_ux{k} + K_t{k}' * Q_uu{k} * K_t{k};
            %3*3
            V_xt{k} = Q_xt{k} + Q_xu{k} * M_t{k} + K_t{k}' * Q_ut{k} + K_t{k}' * Q_uu{k} * M_t{k};
            %3*1
            V_tt{k} = Q_tt{k} + Q_tu{k} * M_t{k} + M_t{k}' * Q_ut{k} + M_t{k}' * Q_uu{k} * M_t{k};
            %1*1
        end

        % Update time parameter using gradient descent 使用梯度下降更新时间参数
        dtheta=ddpack.checkedTimeSolve(V_tt{1},-V_t{1});
        if isempty(costHistory)
            costHistory=J(i);
            meritWeight=10+abs(J(i))/((1-rho)*max(1,norm(x{N}-x_star)));
            meritHistory=NaN;
        end
        direction=struct('feedforward',{k_t},'feedback',{K_t},'time_gain',{M_t},'time_change',dtheta);
        geometry=struct('dimension',2,'centers',obs_position,'radii',obs_radius,'gain',gamma_h);
        [candidateX,candidateU,candidateTime,trial]=ddpack.defectLineSearch( ...
            x,u,t_f,x_star,dyn,cost,u_max,direction,epsilon,t_min,t_max,meritWeight,update_epsilon,geometry);
        lineSearchTrials=[lineSearchTrials;[repmat(i,size(trial.trials,1),1),trial.trials]]; %#ok<AGROW>
        if numel(meritHistory)==1 && isnan(meritHistory(1)),meritHistory(1)=trial.merit_before;end
        if ~trial.accepted
            lineSearchStalled=true;
            break;
        end
        x=candidateX;u=candidateU;t_f=candidateTime;x_new=x;
        d=x{N}-x_star;acceptedSteps=acceptedSteps+1;
        costHistory(end+1,1)=trial.cost_after; %#ok<AGROW>
        meritHistory(end+1,1)=trial.merit_after; %#ok<AGROW>
        catch exception
            if isempty(lastFinite),rethrow(exception);end
            saved=lastFinite;
            sol=ddpack.numericalFailureSolution(saved,exception,numerical);
            return;
        end
    end

    %% Assemble and return solution structure 组装并返回解决方案结构
    
    fprintf("finished geometry-aware safety-corrected PDDP, assembling results for post-processing...\n");  % 完成geometry-aware safety-corrected PDDP，组装结果用于后处理...
    % t = linspace(0.0, s, N);      % Generate time vector (commented out) 生成时间向量 (已注释)
    % Assemble solution 组装解决方案
    sol = assemble_solution(x, u, t, J, E, M, Theta, Q_u, Q_uu, Q_ux, 0,d, dist_to_obs_history,h_x_history, completedIterations);
    sol.final_time = t_f;
    sol.J=costHistory(:);sol.J_accepted=costHistory(:);sol.M=meritHistory(:);
    sol.Theta=Theta(1:completedIterations);sol.E=E(1:completedIterations);
    if numel(costHistory)>completedIterations
        finalEnergy=0;
        for energyIndex=1:N-1,finalEnergy=finalEnergy+t_f*.5*(u{energyIndex}.'*u{energyIndex})*dt;end
        sol.Theta(end+1,1)=t_f;sol.E(end+1,1)=finalEnergy;
    end
    sol.iter=completedIterations;sol.initial_cost=costHistory(1);sol.returned_cost=costHistory(end);
    sol.line_search=struct('method','bounded_actual_merit_backtracking','weight',meritWeight, ...
        'accepted_steps',acceptedSteps,'rejected_trials',sum(lineSearchTrials(:,end)==0), ...
        'trial_columns',{{'iteration','attempt','fraction','epsilon','time_before','time_after','weight', ...
        'cost_before','cost_after','defect_before','defect_after','merit_before','merit_after','geometry_before','geometry_after','geometry_weight','finite','accepted'}}, ...
        'trials',lineSearchTrials);
    if lineSearchStalled
        sol.termination_reason='line_search_no_descent';sol.convergence_status='line_search_stalled';
    elseif converged
        sol.termination_reason='cost_change_below_tolerance';sol.convergence_status='converged_by_cost_change';
    else
        sol.termination_reason='maximum_iterations_reached';sol.convergence_status='maximum_iterations_reached_without_convergence';
    end
    sol.numerical_diagnostics=numerical;
end

%% Helper Functions for geometry-aware safety-corrected PDDP Algorithm geometry-aware safety-corrected PDDP算法的辅助函数

% Assembles and returns solution structure containing all optimization results 组装并返回包含所有优化结果的解决方案结构
%
% This function packages all the computed data from the geometry-aware safety-corrected PDDP algorithm into 此函数将geometry-aware safety-corrected PDDP算法中的所有计算数据打包成
% a structured format for easy access and post-processing. 结构化格式以便于访问和后处理。
%
% Inputs 输入参数
%
% x                   : locally optimal state trajectory (cell array) 局部最优状态轨迹 (单元数组)
% u                   : locally optimal control sequence (cell array) 局部最优控制序列 (单元数组)
% t                   : discretized time stamps (vector) 离散化时间戳 (向量)
% J                   : iteration history of cost function (vector) 代价函数的迭代历史 (向量)
% E                   : iteration history of control energy (vector) 控制能量的迭代历史 (向量)
% M                   : iteration history of merit function (vector) 价值函数的迭代历史 (向量)
% Theta               : iteration history of time parameter (vector) 时间参数的迭代历史 (向量)
% Q_u, Q_uu, Q_ux     : derivatives of state action value function (cell arrays) 状态动作值函数的导数 (单元数组)
% error               : error flag (0 if no error, 1 if error occurred) 错误标志 (无错误为0，发生错误为1)
% d                   : defect vector for terminal-defect correction 多重射击的缺陷向量
% dist_to_obs_history : distance to obstacles history 到障碍物距离历史
% h_x_history         : geometric safety value values history geometric-safety约束值历史
% iter                : number of iterations completed (integer) 完成的迭代次数 (整数)
%
% Outputs 输出参数
%
% sol : solution structure containing all optimization results 包含所有优化结果的解决方案结构
function sol = assemble_solution(x, u, t, J, E, M, Theta, Q_u, Q_uu, Q_ux, error, d, dist_to_obs_history,h_x_history, iter)

    % Create solution structure with all optimization results 创建包含所有优化结果的解决方案结构
    sol = struct;
    sol.error = error;                    % Error flag indicating success/failure 指示成功/失败的错误标志
    sol.x = x;                            % Optimal state trajectory 最优状态轨迹
    sol.u = u(1:numel(x));                % Controls aligned with state nodes
    sol.t = t;                            % Time vector 时间向量
    sol.dt = t(2) - t(1);                 % Time step size 时间步长
    sol.J = J;                            % Cost function history 代价函数历史
    sol.E = E;                            % Control energy history 控制能量历史
    sol.M = M;                            % Merit function history 价值函数历史
    sol.Theta = Theta;                    % Time parameter history 时间参数历史
    sol.Q_u = Q_u;                        % Q-function gradient w.r.t. control Q函数对控制的梯度
    sol.Q_uu = Q_uu;                      % Q-function Hessian w.r.t. control Q函数对控制的Hessian
    sol.Q_ux = Q_ux;                      % Q-function mixed derivative w.r.t. control and state Q函数对控制和状态的混合导数
    sol.d = d;                            % Defect vector 缺陷向量
    % sol.dnorm = dnorm;                  % Defect norm (commented out) 缺陷范数 (已注释)
    % sol.log_d = log_d;                 % Log of defect vectors (commented out) 缺陷向量日志 (已注释)
    sol.dist_to_obs = dist_to_obs_history;  % Distance to obstacles history 到障碍物距离历史
    sol.h_x = h_x_history;                % geometric safety value values history geometric-safety约束值历史
    sol.iter = iter;                      % Number of iterations completed 完成的迭代次数
    return
end

% Updates defect weight based on cost change and defect norm 基于代价变化和缺陷范数更新缺陷权重
%
% This function implements an adaptive weight update strategy for the merit function 此函数实现价值函数的自适应权重更新策略
% to balance between cost minimization and constraint satisfaction. 以平衡代价最小化和约束满足。
%
% Inputs 输入参数
%
% cost_first_order : expected first-order cost change 期望一阶代价变化
% dfectnorm        : current defect norm 当前缺陷范数
% weight_prev      : previous defect weight 前一个缺陷权重
% rho              : update parameter (0 < rho < 1) 更新参数 (0 < rho < 1)
%
% Outputs 输出参数
%
% weight : updated defect weight 更新的缺陷权重
function weight = UpdateDefectWeight(cost_first_order, dfectnorm, weight_prev,rho)

if dfectnorm == 0
    weight = weight_prev;  % No change if no defect 如果无缺陷则无变化
    return;
end

exp_change_abs = abs(cost_first_order);  % Absolute expected change 绝对期望变化
thresh = 10 + exp_change_abs/((1-rho)*dfectnorm);  % Threshold for weight update 权重更新阈值

if weight_prev >= thresh
    weight = weight_prev;  % Keep previous weight if above threshold 如果超过阈值则保持前一个权重
else
    weight = thresh;       % Update to threshold value 更新到阈值
end

weight = max(10, weight);  % Ensure minimum weight 确保最小权重

end

% Computes the norm of the defect vector 计算缺陷向量的范数
%
% This function calculates the specified norm of the defect vector 此函数计算缺陷向量的指定范数
% for use in the merit function. 用于价值函数。
%
% Inputs 输入参数
%
% d       : defect vector 缺陷向量
% norm_id : norm type (1 for L1, 2 for L2, etc.) 范数类型 (1为L1，2为L2，等)
%
% Outputs 输出参数
%
% cost : computed norm value 计算的范数值
function cost = ComputeDefectNorm(d, norm_id)
    norms = norm(d, norm_id);     % Calculate the specified norm of the vector 计算向量的指定范数
    cost = norm(norms, norm_id);  % Calculate the specified norm again for all norm results 对所有范数结果再次计算指定范数
% % end
end

% Generates initial trajectory using proportional guidance 使用比例制导生成初始轨迹
%
% This function creates a smooth initial trajectory from start to end point 此函数创建从起点到终点的平滑初始轨迹
% using proportional navigation guidance law. 使用比例导航制导律。
%
% Inputs 输入参数
%
% statr_point : starting position [x, y, theta] 起始位置 [x, y, theta]
% end_point   : target position [x, y, theta] 目标位置 [x, y, theta]
% tf          : time horizon 时间范围
% ~           : unused parameter (placeholder) 未使用参数 (占位符)
%
% Outputs 输出参数
%
% path    : generated trajectory points (cell array) 生成的轨迹点 (单元数组)
% control : generated control sequence (cell array) 生成的控制序列 (单元数组)
function [path, control] = Proportional_guidance(statr_point, end_point, tf, ~)
%UNTITLED2 Summary of this function is displayed here 此处显示此函数的摘要
%   Detailed description is displayed here 此处显示详细描述

vel = [0.0; 0.0; 0.0];              % Initial velocity [v_x,v_y,q_dot] 初始速度 [v_x,v_y,q_dot]
pos = [statr_point(1); statr_point(2); statr_point(3)];  % Initial position [x,y,gamma] 初始位置 [x,y,gamma]
q_dot = 0;                          % Initial angular velocity 初始角速度
% path = [];                        % Trajectory storage (commented out) 轨迹存储 (已注释)
% control = [];                     % Control storage (commented out) 控制存储 (已注释)

% Proportional gain for guidance law 制导律的比例增益
K = 5;

dt = 0.1;                           % Time step for guidance 制导的时间步长

N = floor(tf ./ dt);                % Number of time steps 时间步数

path = cell(N, 1);                  % Initialize trajectory storage 初始化轨迹存储
control = cell(N, 1);               % Initialize control storage 初始化控制存储

for k = 1:N-1
    % Target and missile velocity components 目标和导弹速度分量
    V_Ty = 0;                       % Target velocity in y 目标y方向速度
    V_My = vel(2);                  % Missile velocity in y 导弹y方向速度
    V_Tx = 0;                       % Target velocity in x 目标x方向速度
    V_Mx = vel(1);                  % Missile velocity in x 导弹x方向速度

    % Target and missile positions 目标和导弹位置
    Tx = end_point(1);              % Target x position 目标x位置
    Mx = pos(1);                    % Missile x position 导弹x位置
    Ty = end_point(2);              % Target y position 目标y位置
    My = pos(2);                    % Missile y position 导弹y位置
    
    % path = [path, pos];           % Store position (commented out) 存储位置 (已注释)
    % control = [control, K*q_dot]; % Store control (commented out) 存储控制 (已注释)
    path{k} = pos;                  % Store current position 存储当前位置
    control{k} = K * q_dot;         % Store current control 存储当前控制
    
    % Proportional navigation guidance law 比例导航制导律
    q_dot = ( ( V_Ty - V_My )*( Tx - Mx ) - ( Ty - My )*( V_Tx - V_Mx ) ) / ( (Tx - Mx)^2 + (Ty - My)^2 );
    
    % Update position and velocity 更新位置和速度
    pos = pos + vel .* dt;          % Update position 更新位置
    vel(1) = 1.0 * cos(pos(3));     % Update x velocity 更新x速度
    vel(2) = 1.0 * sin(pos(3));     % Update y velocity 更新y速度
    vel(3) = K*q_dot;               % Update angular velocity 更新角速度
end

% path = [path, pos];               % Store final position (commented out) 存储最终位置 (已注释)
% control = [control, K*q_dot];     % Store final control (commented out) 存储最终控制 (已注释)
path{N} = pos;                      % Store final position 存储最终位置
control{N} = K * q_dot;             % Store final control 存储最终控制

end
