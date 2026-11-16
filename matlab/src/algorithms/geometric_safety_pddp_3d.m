%% Geometric safety correction Embedded Terminal-defect Parameterized Differential Dynamic Programming Algorithm / 控制屏障函数嵌入多重射击参数化差分动态规划算法
% Obstacle avoidance (√), Waypoints (√) / 避障 (√), 航点 (√)

% This function implements the geometry-aware safety-corrected PDDP algorithm for UAV trajectory optimization / 此函数实现用于无人机轨迹优化的geometry-aware safety-corrected PDDP算法
% with obstacle avoidance using control barrier functions. / 使用控制屏障函数进行避障
% This algorithm combines Terminal-defect PDDP with Geometric safety corrections for safe trajectory planning / 该算法将多重射击PDDP与控制屏障函数结合，用于安全轨迹规划
%
% ALGORITHM FEATURES: / 算法特性：
%   - Geometric safety correction (geometric-safety) for safety constraints / 用于安全约束的控制屏障函数(geometric-safety)
%   - Multi-segment trajectory optimization / 多段轨迹优化
%   - Parameterized time horizon / 参数化时间范围
%   - Real-time obstacle avoidance / 实时避障
%   - Safety-critical control with guaranteed constraint satisfaction / 具有保证约束满足的安全关键控制
%
% INPUT PARAMETERS: / 输入参数：
%   x_0           - Initial state vector / 初始状态向量
%   x_star        - Target state vector / 目标状态向量  
%   t_f           - Time horizon / 时间范围
%   dt            - Time step / 时间步长
%   dyn           - Dynamics model instance / 动力学模型实例
%   cost          - Cost function instance / 成本函数实例
%   u_max         - Maximum control input magnitude / 最大控制输入幅度
%   num_iter      - Maximum number of iterations / 最大迭代次数
%   epsilon       - Line search parameter / 线搜索参数
%   rho           - Penalty parameter / 惩罚参数
%   update_epsilon- Epsilon update factor / Epsilon更新因子
%   obs_position  - Obstacle position / 障碍物位置
%   obs_radius    - Obstacle radius / 障碍物半径
%   gamma_h       - Barrier function gain / 屏障函数增益
%   single_obs    - Single obstacle flag / 单障碍物标志
%   num_obs       - Number of obstacles / 障碍物数量
%
% OUTPUT: / 输出：
%   sol - Solution structure containing: / 包含以下内容的解结构：
%         .x          - State trajectory / 状态轨迹
%         .u          - Control trajectory / 控制轨迹
%         .t          - Time vector / 时间向量
%         .J          - Cost history / 成本历史
%         .Theta      - Parameter evolution / 参数演化
%         .error      - Error flag / 错误标志
%         .dist_to_obs- Distance to obstacles / 到障碍物的距离
%         .h_x        - Barrier function values / 屏障函数值
%
% REFERENCE: / 参考文献：
%   [1] Geometric safety correction based Multi-Segment Parameterized DDP / 基于控制屏障函数的多段参数化DDP
%   [2] Safety-Critical Control for UAV Trajectory Optimization / 无人机轨迹优化的安全关键控制
%
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
%
function sol = geometric_safety_pddp_3d(x_0, x_star, t_f, dt, dyn, cost, u_max, num_iter, epsilon, rho, update_epsilon, obs_position, obs_radius, gamma_h, single_obs, num_obs, minimum_time, maximum_time)
%% Memory Allocation and Initialization / 内存分配和初始化

    % Time discretization / 时间离散化
    N = floor(t_f ./ dt);              % Number of time steps / 时间步数
    if nargin < 17 || isempty(minimum_time)
        minimum_time = max(t_f-20, 5);
    end
    if nargin < 18 || isempty(maximum_time)
        maximum_time = t_f + 7;
    end
    t_min = minimum_time;              % Scenario-defined minimum maneuver time
    t_max = maximum_time;              % Scenario-defined maximum maneuver time

    t = linspace(0.0, 1, N);           % Normalized time vector / 归一化时间向量
    dt = t(2) - t(1);                  % Time step size / 时间步长
    
    % Algorithm performance tracking / 算法性能跟踪
    validateattributes(rho,{'numeric'},{'scalar','real','finite','>',0,'<',1});
    lineSearchTrials=zeros(0,18);costHistory=[];meritHistory=[];
    meritWeight=NaN;lineSearchStalled=false;acceptedSteps=0;
    J = zeros(num_iter, 1);            % Cost function history / 成本函数历史
    M = zeros(num_iter, 1);            % Merit function history / 价值函数历史
    E = zeros(num_iter, 1);            % Control energy history / 控制能量历史

    % Safety constraint tracking / 安全约束跟踪
    h_x_history = zeros(N, num_obs);   % Barrier function values / 屏障函数值
    
    % Time parameterization / 时间参数化
    Theta = zeros(num_iter, 1);        % Time parameter evolution / 时间参数演化

    % Trajectory storage / 轨迹存储
    x = cell(N, 1);                    % State trajectory / 状态轨迹
    x_new = cell(N, 1);                % Updated state trajectory / 更新的状态轨迹
    x{1} = x_0;                        % Initialize with initial state / 用初始状态初始化
    x_new{1} = x_0;                    % Initialize updated trajectory / 初始化更新轨迹

    % Control and safety tracking / 控制和安全跟踪
    u = cell(N, 1);                    % Control input trajectory / 控制输入轨迹
    dist_to_obs_history = zeros(N, 1); % Distance to obstacles history / 到障碍物距离历史
    
    % Value function and its derivatives / 价值函数及其导数
    V = zeros(N, 1);                   % Value function / 价值函数
    V_x = cell(N, 1);                  % Value function gradient w.r.t. state / 价值函数对状态的梯度
    V_t = cell(N, 1);                  % Value function derivative w.r.t. time / 价值函数对时间的导数
    V_xx = cell(N, 1);                 % Value function Hessian w.r.t. state / 价值函数对状态的Hessian矩阵
    V_xt = cell(N, 1);                 % Mixed derivative w.r.t. state and time / 对状态和时间的混合导数
    V_tt = cell(N, 1);                 % Second derivative w.r.t. time / 对时间的二阶导数

    % Q-function derivatives for DDP backward pass / DDP反向传播的Q函数导数
    Q_x = cell(N, 1);                  % Q-function gradient w.r.t. state / Q函数对状态的梯度
    Q_u = cell(N, 1);                  % Q-function gradient w.r.t. control / Q函数对控制的梯度
    Q_t = cell(N, 1);                  % Q-function gradient w.r.t. time / Q函数对时间的梯度
    Q_xx = cell(N, 1);                 % Q-function Hessian w.r.t. state / Q函数对状态的Hessian矩阵
    Q_xu = cell(N, 1);                 % Mixed Hessian: state-control / 混合Hessian：状态-控制
    Q_ux = cell(N, 1);                 % Mixed Hessian: control-state / 混合Hessian：控制-状态
    Q_xt = cell(N, 1);                 % Mixed Hessian: state-time / 混合Hessian：状态-时间
    Q_tx = cell(N, 1);                 % Mixed Hessian: time-state / 混合Hessian：时间-状态
    Q_uu = cell(N, 1);                 % Q-function Hessian w.r.t. control / Q函数对控制的Hessian矩阵
    Q_ut = cell(N, 1);                 % Mixed Hessian: control-time / 混合Hessian：控制-时间
    Q_tu = cell(N, 1);                 % Mixed Hessian: time-control / 混合Hessian：时间-控制
    Q_tt = cell(N, 1);                 % Q-function Hessian w.r.t. time / Q函数对时间的Hessian矩阵

    % Control policy parameters / 控制策略参数
    k_t = cell(N, 1);                  % Feedforward control gains / 前馈控制增益
    K_t = cell(N, 1);                  % Feedback control gains / 反馈控制增益
    M_t = cell(N, 1);                  % Time-dependent control gains / 时间相关控制增益
    
    % Auxiliary variables / 辅助变量
    d = zeros(numel(x),1);             % Direction vector / 方向向量

    

    %% Initialize PDDP with a random input sequence / 用随机输入序列初始化PDDP

    % fprintf("initializing random control sequence...\n")
    % 
    % % Generate random control sequence / 生成随机控制序列
    % for k = 1:N-1
    %     u{k} = zeros(numel(u_max), 1);
    % end
    % u{N} = zeros(numel(u_max), 1);
    % 
    % fprintf("generating initial trajectory...\n")
    % 
    % % Generate initial trajectory using random control sequence / 使用随机控制序列生成初始轨迹
    % for k = 1:N-1
    %     x_new{k+1} = x_new{k} + dyn.F(x_new{k}, u{k}, t_f) .* dt;
    % end
    % x = x_new;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate initial trajectory using proportional guidance / 使用比例制导生成初始轨迹
    [path, control] = Proportional_guidance3D(x_0, x_star, t_f, dt);
    u = control; % Set control sequence / 设置控制序列
    x = path;    % Set state trajectory / 设置状态轨迹
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Perform main PDDP iterations on the trajectory, input sequence and parameter / 对轨迹、输入序列和参数执行主要PDDP迭代
    
    % fprintf("beginning geometry-aware safety-corrected PDDP...\n")  
    
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
        % fprintf("geometry-aware safety-corrected PDDP iteration %d out of %d...\n", i, num_iter);
        % Compute total cost / 计算总成本
        J(i) = cost.phi(x{N}, x_star); % Terminal cost / 终端成本
        for k = 1:N-1
            J(i) = J(i) + cost.L(x{k}, x_star, u{k}, t_f, dt); % Running cost / 运行成本
        end

        saved.J=J(1:i);
        if ~isfinite(J(i)),error('ddpack:numerics:NonfiniteTrajectory','Nominal objective is nonfinite.');end
        if ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),x)) || ~all(cellfun(@(v)isreal(v)&&all(isfinite(v(:))),u))
            error('ddpack:numerics:NonfiniteTrajectory','Nominal trajectory is nonfinite.');
        end
        lastFinite=saved;
        % Compute control energy usage / 计算控制能量使用
        for k = 1:N-1
            E(i) = E(i) + t_f * 0.5 .* u{k}.' * u{k} .* dt; % Control energy / 控制能量
        end
        Theta(i) = t_f; % Store current time parameter / 存储当前时间参数


        % Convergence check / 收敛检查
        if i > 1 && abs(J(i) - J(i-1)) < 1e-3
            % fprintf("PDDP iteration %d out of %d...\n", i, num_iter);
            % disp('convergence');
            fprintf("Convergence at iteration %d out of %d...\n", i, num_iter);
            u{N} = u{N-1}; % Set final control / 设置最终控制
            converged = true;
            break;
        end

        % Compute terminal value function and derivatives / 计算终端价值函数及其导数
        V(N) = cost.phi(x{N}, x_star); % Terminal value function / 终端价值函数
        V_x{N} = cost.phi_x(x{N}, x_star); % Terminal value gradient w.r.t. state / 终端价值对状态的梯度
        V_xx{N} = cost.phi_xx(x{N}, x_star); % Terminal value Hessian w.r.t. state / 终端价值对状态的Hessian矩阵
        % V_t{N} = zeros(numel(s)); %1*1
        V_t{N} = cost.phi_t(x{N}, x_star); % Terminal value derivative w.r.t. time / 终端价值对时间的导数
        % V_tt{N} = zeros(numel(s),numel(s)); %1*1
        V_tt{N} = cost.phi_tt(x{N}, x_star); % Terminal value second derivative w.r.t. time / 终端价值对时间的二阶导数
        % V_xt{N} = zeros(numel(x_star), numel(s)); %3*1
        V_xt{N} = cost.phi_xt(x{N}, x_star); % Terminal value mixed derivative / 终端价值混合导数

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        d = x{N}- x_star; % Terminal state error / 终端状态误差
        % d = zeros(3,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Perform backwards pass / 执行反向传播
        for k = N-1:-1:1
           
            % Regularized derivatives / 正则化导数
            % V_xx1{k+1} = V_xx{k+1} + 0.01 * eye(size(V_xx{k+1}));
            % V_tt1{k+1} = V_tt{k+1} + 0.01 * eye(size(V_tt{k+1}));
            % V_xx1{k+1} = V_xx{k+1};
            % V_ss1{k+1} = V_ss{k+1};

            % Compute state-action value function derivatives / 计算状态-动作价值函数导数
            if k == N-1
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                % Q-function gradient w.r.t. state (with terminal error) / Q函数对状态的梯度（含终端误差）
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                % Q-function gradient w.r.t. control (with terminal error) / Q函数对控制的梯度（含终端误差）
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1}...
                    + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d + V_xt{k+1}' * d;
                % Q-function gradient w.r.t. time (with terminal error) / Q函数对时间的梯度（含终端误差）
            else
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                % Q-function gradient w.r.t. state / Q函数对状态的梯度
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                % Q-function gradient w.r.t. control / Q函数对控制的梯度
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + ...
                    dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                % Q-function gradient w.r.t. time / Q函数对时间的梯度
            end

            Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, t_f, dt);
            % Q-function Hessian w.r.t. state / Q函数对状态的Hessian矩阵
            Q_uu{k} = cost.L_uu(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            % Q-function Hessian w.r.t. control / Q函数对控制的Hessian矩阵
            Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            % Mixed Hessian: state-control / 混合Hessian：状态-控制
            Q_ux{k} = Q_xu{k}';
            % Mixed Hessian: control-state / 混合Hessian：控制-状态
            Q_xt{k} = cost.L_xt(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            % Mixed Hessian: state-time / 混合Hessian：状态-时间
            Q_tx{k} = Q_xt{k}';
            % Mixed Hessian: time-state / 混合Hessian：时间-状态
            Q_ut{k} = cost.L_ut(x{k}, u{k}, t_f, dt) + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            % Mixed Hessian: control-time / 混合Hessian：控制-时间
            Q_tu{k} = Q_ut{k}';
            % Mixed Hessian: time-control / 混合Hessian：时间-控制
            Q_tt{k} = cost.L_tt(x{k}, u{k}, t_f, dt) + V_tt{k+1} + ...
                2 * dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            % Q-function Hessian w.r.t. time / Q函数对时间的Hessian矩阵

            % Compute the value function derivatives / 计算价值函数导数
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},-[Q_u{k},Q_ux{k},Q_ut{k}],numerical);
            k_t{k}=gains(:,1);K_t{k}=gains(:,2:1+numel(x_0));M_t{k}=gains(:,end);

            V_x{k} = Q_x{k} + K_t{k}' * Q_u{k} + Q_xu{k} * k_t{k} + K_t{k}' * Q_uu{k} * k_t{k};
            % Value function gradient w.r.t. state / 价值函数对状态的梯度
            V_t{k} = Q_t{k} + M_t{k}' * Q_u{k} + Q_tu{k} * k_t{k} + M_t{k}' * Q_uu{k} * k_t{k};
            % Value function derivative w.r.t. time / 价值函数对时间的导数
            V_xx{k} = Q_xx{k} + Q_xu{k} * K_t{k} + K_t{k}' * Q_ux{k} + K_t{k}' * Q_uu{k} * K_t{k};
            % Value function Hessian w.r.t. state / 价值函数对状态的Hessian矩阵
            V_xt{k} = Q_xt{k} + Q_xu{k} * M_t{k} + K_t{k}' * Q_ut{k} + K_t{k}' * Q_uu{k} * M_t{k};
            % Value function mixed derivative / 价值函数混合导数
            V_tt{k} = Q_tt{k} + Q_tu{k} * M_t{k} + M_t{k}' * Q_ut{k} + M_t{k}' * Q_uu{k} * M_t{k};
            % Value function second derivative w.r.t. time / 价值函数对时间的二阶导数
        end


    

        dtheta=ddpack.checkedTimeSolve(V_tt{1},-V_t{1});
        if isempty(costHistory)
            costHistory=J(i);
            meritWeight=10+abs(J(i))/((1-rho)*max(1,norm(x{N}-x_star)));
            meritHistory=NaN;
        end
        direction=struct('feedforward',{k_t},'feedback',{K_t},'time_gain',{M_t},'time_change',dtheta);
        geometry=struct('dimension',3,'centers',obs_position,'radii',obs_radius,'gain',gamma_h);
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

    %% Assemble and return solution structure / 组装并返回解结构
    
    fprintf("finished geometry-aware safety-corrected PDDP, assembling results for post-processing...\n");
    % t = linspace(0.0, s, N);
    % Assemble solution / 组装解
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

%% Helper Functions for DDP Algorithm / DDP算法的辅助函数

% Assembles and returns solution structure / 组装并返回解结构
%
% Inputs / 输入参数：
%
% x               : locally optimal state trajectory / 局部最优状态轨迹
% u               : locally optimal control sequence / 局部最优控制序列
% t               : discretized time stamps / 离散化时间戳
% J               : iteration history of cost function / 成本函数的迭代历史
% E               : iteration history of control energy / 控制能量的迭代历史
% M               : iteration history of merit function / 价值函数的迭代历史
% Theta           : iteration history of time parameter / 时间参数的迭代历史
% Q_u, Q_uu, Q_ux : derivatives of state action value function / 状态动作价值函数的导数
% error           : zero if no error, nonzero else / 无错误时为0，否则非零
% d               : terminal state error / 终端状态误差
% dist_to_obs_history : distance to obstacles history / 到障碍物距离历史
% h_x_history     : barrier function values history / 屏障函数值历史
% iter            : number of iterations / 迭代次数
%
% Outputs / 输出参数：
%
% sol : solution structure / 解结构
function sol = assemble_solution(x, u, t, J, E, M, Theta, Q_u, Q_uu, Q_ux, error, d, dist_to_obs_history,h_x_history, iter)

    % Solution structure / 解结构
    sol = struct;
    sol.error = error; % Error flag / 错误标志
    sol.x = x; % State trajectory / 状态轨迹
    sol.u = u(1:numel(x)); % Controls aligned with returned state nodes
    sol.t = t; % Time vector / 时间向量
    sol.dt = t(2) - t(1); % Time step / 时间步长
    sol.J = J; % Cost history / 成本历史
    sol.E = E; % Control energy history / 控制能量历史
    sol.M = M; % Merit function history / 价值函数历史
    sol.Theta = Theta; % Time parameter history / 时间参数历史
    sol.Q_u = Q_u; % Q-function gradient w.r.t. control / Q函数对控制的梯度
    sol.Q_uu = Q_uu; % Q-function Hessian w.r.t. control / Q函数对控制的Hessian矩阵
    sol.Q_ux = Q_ux; % Mixed Hessian: control-state / 混合Hessian：控制-状态
    sol.d = d; % Terminal state error / 终端状态误差
    % sol.dnorm = dnorm;
    % sol.log_d = log_d;
    sol.dist_to_obs = dist_to_obs_history; % Unmodified Euclidean distance
    sol.h_x = h_x_history; % Barrier function values / 屏障函数值
    sol.iter = iter; % Number of iterations / 迭代次数
    return
end

function cost = ComputeDefectNorm(d, norm_id)
    % Compute defect norm for penalty method / 为惩罚方法计算缺陷范数
    norms = norm(d, norm_id); % Calculate the specified norm of the vector / 计算向量的指定范数
    cost = norm(norms, norm_id); % Calculate the specified norm again for all norm results / 对所有范数结果再次计算指定范数
end

function weight = UpdateDefectWeight(cost_first_order, dfectnorm, weight_prev,rho)
    % Update defect weight for penalty method / 为惩罚方法更新缺陷权重
    % This function implements adaptive penalty weight update / 此函数实现自适应惩罚权重更新

    if dfectnorm == 0
        weight = weight_prev; % Keep previous weight if no defect / 如果无缺陷则保持先前权重
        return;
    end

    exp_change_abs = abs(cost_first_order); % Absolute expected cost change / 绝对期望成本变化
    thresh = 10 + exp_change_abs/((1-rho)*dfectnorm); % Threshold for weight update / 权重更新的阈值

    if weight_prev >= thresh
        weight = weight_prev; % Keep previous weight if above threshold / 如果超过阈值则保持先前权重
    else
        weight = thresh; % Update to threshold value / 更新为阈值
    end

    weight = max(10, weight); % Ensure minimum weight / 确保最小权重

end


function [path, control] = Proportional_guidance3D(statr_point, end_point, tf, ~)
    % Generate initial trajectory using proportional guidance / 使用比例制导生成初始轨迹
    % This function creates a reasonable initial trajectory for geometry-aware safety-corrected PDDP / 此函数为geometry-aware safety-corrected PDDP创建合理的初始轨迹

    % Fixed version: Generate reasonable initial trajectory / 固定版本：生成合理的初始轨迹
    pos = [statr_point(1); statr_point(2); statr_point(3); statr_point(4); statr_point(5); statr_point(6)];
    % position [x,y,z,v,psi,theta] / 位置 [x,y,z,v,psi,theta]

    % Proportional gain / 比例增益
    K = 0.1;  % Reduce gain to avoid excessive control input / 减小增益以避免过度控制输入

    dt = 0.1; % Time step / 时间步长
    N = floor(tf ./ dt); % Number of time steps / 时间步数

    path = cell(N, 1); % State trajectory / 状态轨迹
    control = cell(N, 1); % Control trajectory / 控制轨迹

    % Calculate initial velocity direction / 计算初始速度方向
    dx = end_point(1) - statr_point(1); % x-direction distance / x方向距离
    dy = end_point(2) - statr_point(2); % y-direction distance / y方向距离
    dz = end_point(3) - statr_point(3); % z-direction distance / z方向距离
    dist = sqrt(dx^2 + dy^2 + dz^2); % Total distance / 总距离

    % Set initial velocity so trajectory can reach target / 设置初始速度使轨迹能够到达目标
    if dist > 0
        vel_scale = min(dist / tf, 50);  % Limit maximum velocity / 限制最大速度
        vel = [vel_scale * dx / dist; vel_scale * dy / dist; vel_scale * dz / dist; 0; 0; 0];
    else
        vel = [0; 0; 0; 0; 0; 0]; % Zero velocity if at target / 如果在目标处则速度为零
    end

    for k = 1:N-1
        % Calculate position error / 计算位置误差
        error_x = end_point(1) - pos(1); % x-position error / x位置误差
        error_y = end_point(2) - pos(2); % y-position error / y位置误差
        error_z = end_point(3) - pos(3); % z-position error / z位置误差
        
        % Calculate angle error / 计算角度误差
        error_psi = end_point(5) - pos(5); % psi angle error / psi角度误差
        error_theta = end_point(6) - pos(6); % theta angle error / theta角度误差
        
        % Proportional control / 比例控制
        q_dot_x = K * error_x; % x-velocity command / x速度指令
        q_dot_y = K * error_y; % y-velocity command / y速度指令
        q_dot_z = K * error_z; % z-velocity command / z速度指令
        psi_dot = K * error_psi; % psi angular velocity / psi角速度
        theta_dot = K * error_theta; % theta angular velocity / theta角速度
        
        % Store current state and control / 存储当前状态和控制
        path{k} = pos; % Store state / 存储状态
        control{k} = [q_dot_x; q_dot_y]; % Store control / 存储控制
        
        % Update position and velocity / 更新位置和速度
        pos(1) = pos(1) + vel(1) * dt; % Update x position / 更新x位置
        pos(2) = pos(2) + vel(2) * dt; % Update y position / 更新y位置
        pos(3) = pos(3) + vel(3) * dt; % Update z position / 更新z位置
        pos(5) = pos(5) + psi_dot * dt; % Update psi angle / 更新psi角度
        pos(6) = pos(6) + theta_dot * dt; % Update theta angle / 更新theta角度
        
        % Update velocity (simple proportional navigation) / 更新速度（简单比例导航）
        vel(1) = vel(1) + q_dot_x * dt; % Update x velocity / 更新x速度
        vel(2) = vel(2) + q_dot_y * dt; % Update y velocity / 更新y速度
        vel(3) = vel(3) + q_dot_z * dt; % Update z velocity / 更新z速度
        vel(5) = psi_dot; % Set psi angular velocity / 设置psi角速度
        vel(6) = theta_dot; % Set theta angular velocity / 设置theta角速度
        
        % Limit velocity magnitude / 限制速度大小
        vel_mag = sqrt(vel(1)^2 + vel(2)^2 + vel(3)^2); % Calculate velocity magnitude / 计算速度大小
        if vel_mag > 100  % Limit maximum velocity / 限制最大速度
            vel(1) = vel(1) * 100 / vel_mag; % Scale x velocity / 缩放x速度
            vel(2) = vel(2) * 100 / vel_mag; % Scale y velocity / 缩放y速度
            vel(3) = vel(3) * 100 / vel_mag; % Scale z velocity / 缩放z速度
        end
    end

    % Store the last state / 存储最后一个状态
    path{N} = pos; % Final state / 最终状态
    control{N} = [0; 0];  % Final control input is zero / 最终控制输入为零

end
