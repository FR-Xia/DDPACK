%% Terminal-defect Parameterized Differential Dynamic Programming Algorithm / 多重射击参数化差分动态规划算法
% Obstacle avoidance (×), Waypoints (√) / 避障 (×), 航点 (√)

% Performs Terminal-defect PDDP Algorithm to produce a locally optimal control sequence / 
% 执行多重射击PDDP算法以产生局部最优控制序列
%
% Inputs / 输入参数
% 
% x_0      : initial state vector / 初始状态向量
% x_star   : target state vector / 目标状态向量
% t_f      : time horizon / 时间范围
% dt       : time step / 时间步长
% dyn      : instance of Dynamics class / 动力学类实例
% cost     : instance of Cost class / 成本类实例
% u_max    : maximum magnitude of control, used for clamping / 控制的最大幅值，用于限制
% num_iter : number of PDDP iterations / PDDP迭代次数
% epsilon  : PDDP learning parameter for line searching / PDDP线搜索学习参数
% rho      : parameter for defect weight update / 缺陷权重更新参数
% update_epsilon : parameter for epsilon update / epsilon更新参数
%
% Outputs / 输出参数
% 
% sol : structure with solution components / 包含解组件的结构体
%
function sol = terminal_defect_pddp(x_0, x_star, t_f, dt, dyn, cost, u_max, num_iter, epsilon, rho, update_epsilon)
%% Allocate arrays for PDDP / 为PDDP分配数组

    % Time stamp array and timestep / 时间戳数组和时间步长
    N = floor(t_f ./ dt);        % Number of time discretization points / 时间离散化点数
    t_max = t_f + 20;            % Maximum time horizon / 最大时间范围
    t_min = max(t_f-20, 5);      % Minimum time horizon / 最小时间范围

    % Normalized time vector from 0 to 1 / 从0到1的归一化时间向量
    t = linspace(0.0, 1, N);
    dt = t(2) - t(1);            % Actual time step / 实际时间步长
    
    % Cost history for tracking convergence / 用于跟踪收敛的成本历史
    validateattributes(rho,{'numeric'},{'scalar','real','finite','>',0,'<',1});
    lineSearchTrials=zeros(0,18);costHistory=[];meritHistory=[];
    meritWeight=NaN;lineSearchStalled=false;acceptedSteps=0;
    J = zeros(num_iter, 1);      % Total cost history / 总成本历史
    M = zeros(num_iter, 1);      % Merit function history / 价值函数历史
    % merit = cell(N, 1);        % Alternative merit function storage / 替代价值函数存储
    % Control energy history for analysis / 用于分析的控制能量历史
    E = zeros(num_iter, 1);

    % Parameter history / 参数历史
    Theta = zeros(num_iter, 1);  % Time horizon parameter history / 时间范围参数历史

    %
    % xlocal = cell(N, 1);       % Alternative local state storage / 替代局部状态存储

    % State trajectory storage / 状态轨迹存储
    x = cell(N, 1);              % Current state trajectory / 当前状态轨迹
    x_new = cell(N, 1);          % Updated state trajectory / 更新的状态轨迹
    x{1} = x_0;                  % Initialize with initial state / 用初始状态初始化
    x_new{1} = x_0;              % Initialize with initial state / 用初始状态初始化

    % xlocal{1} = x{1};          % Alternative initialization / 替代初始化
    % Control Input trajectory storage / 控制输入轨迹存储
    u = cell(N, 1);              % Control sequence / 控制序列

    % Value function and derivatives storage / 值函数及其导数存储
    V = zeros(N, 1);             % Value function / 值函数
    V_x = cell(N, 1);            % Value function gradient w.r.t. state / 值函数对状态的梯度
    V_t = cell(N, 1);            % Value function derivative w.r.t. time / 值函数对时间的导数
    % V_xx1 = cell(N, 1);        % Alternative regularized Hessian / 替代正则化海塞矩阵
    V_xx = cell(N, 1);           % Value function Hessian w.r.t. state / 值函数对状态的海塞矩阵
    V_xt = cell(N, 1);           % Value function cross derivative / 值函数交叉导数
    V_tt = cell(N, 1);           % Value function second derivative w.r.t. time / 值函数对时间的二阶导数
    % V_tt1 = cell(N, 1);        % Alternative regularized time Hessian / 替代正则化时间海塞矩阵

    % State action value derivatives storage / 状态动作值导数存储
    Q_x = cell(N, 1);            % Q-function gradient w.r.t. state / Q函数对状态的梯度
    Q_u = cell(N, 1);            % Q-function gradient w.r.t. control / Q函数对控制的梯度
    Q_t = cell(N, 1);            % Q-function derivative w.r.t. time / Q函数对时间的导数
    Q_xx = cell(N, 1);           % Q-function Hessian w.r.t. state / Q函数对状态的海塞矩阵
    Q_xu = cell(N, 1);           % Q-function cross derivative w.r.t. state and control / Q函数对状态和控制的交叉导数
    Q_ux = cell(N, 1);           % Q-function cross derivative w.r.t. control and state / Q函数对控制和状态的交叉导数
    Q_xt = cell(N, 1);           % Q-function cross derivative w.r.t. state and time / Q函数对状态和时间的交叉导数
    Q_tx = cell(N, 1);           % Q-function cross derivative w.r.t. time and state / Q函数对时间和状态的交叉导数
    Q_uu = cell(N, 1);           % Q-function Hessian w.r.t. control / Q函数对控制的海塞矩阵
    Q_ut = cell(N, 1);           % Q-function cross derivative w.r.t. control and time / Q函数对控制和时间的交叉导数
    Q_tu = cell(N, 1);           % Q-function cross derivative w.r.t. time and control / Q函数对时间和控制的交叉导数
    Q_tt = cell(N, 1);           % Q-function second derivative w.r.t. time / Q函数对时间的二阶导数

    % Control policy parameters / 控制策略参数
    k_t = cell(N, 1);            % Feed-forward control gain / 前馈控制增益
    K_t = cell(N, 1);            % Feedback control gain matrix / 反馈控制增益矩阵
    M_t = cell(N, 1);            % Time-dependent control gain / 时间相关控制增益
    % log_d = [];                % Alternative defect logging / 替代缺陷记录
    d = zeros(3,1);              % Defect vector for terminal-defect correction / 多重射击的缺陷向量
    % act_merit_change = zeros(num_iter, 1);  % Actual merit change / 实际价值变化
    % exp_cost_change = zeros(num_iter, 1);   % Expected cost change / 预期成本变化
    % exp_merit_change = zeros(num_iter, 1);  % Expected merit change / 预期价值变化

    

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
    % Initialize using proportional guidance / 使用比例制导初始化
    [path, control] = Proportional_guidance(x_0, x_star, t_f, dt);
    u = control;  % Set control sequence from guidance / 从制导设置控制序列
    x = path;     % Set state trajectory from guidance / 从制导设置状态轨迹
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Perform main PDDP iterations on the trajectory, input sequence and parameter / 对轨迹、输入序列和参数执行主要PDDP迭代
    
    fprintf("beginning PDDP...\n")  
    
    % Main PDDP iteration loop / 主要PDDP迭代循环
    x=x(1:N);u=u(1:N);t_f=min(t_max,max(t_min,t_f));
    x{1}=x_0;
    for seedIndex=1:N-1
        u{seedIndex}=min(u_max,max(-u_max,u{seedIndex}));
        x{seedIndex+1}=x{seedIndex}+dt*dyn.F(x{seedIndex},u{seedIndex},t_f);
    end
    u{N}=u{N-1};x_new=x;
    converged=false;completedIterations=0;
    lastFinite=[];
    numerical=struct('linear_solves',0,'regularized_solves',0,'maximum_damping',0,'backward_retries',0);
    for i = 1:num_iter
        saved=struct('x',{x},'u',{u},'t',t,'J',J(1:max(1,i-1)),'iter',i,'final_time',t_f);
        try
        completedIterations=i;
        fprintf("PDDP iteration %d out of %d...\n", i, num_iter);
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
            E(i) = E(i) + t_f * 0.5 .* u{k}.' * u{k} .* dt;
        end
        % Store current time horizon / 存储当前时间范围
        Theta(i) = t_f;


        % Check for convergence / 检查收敛性
        if i > 1 && abs(J(i) - J(i-1)) < 1e-5
            % fprintf("PDDP iteration %d out of %d...\n", i, num_iter);
            % disp('convergence');
            fprintf("Convergence...\n")
            % Set final control to previous control / 将最终控制设置为前一个控制
            u{N} = u{N-1};
            converged=true;
            break;
        end

        % Compute terminal value function and derivatives / 计算终端值函数及其导数
        V(N) = cost.phi(x{N}, x_star);        % Terminal value function (1x1) / 终端值函数 (1x1)
        V_x{N} = cost.phi_x(x{N}, x_star);    % Terminal value gradient (3x1) / 终端值梯度 (3x1)
        V_xx{N} = cost.phi_xx(x{N}, x_star);  % Terminal value Hessian (3x3) / 终端值海塞矩阵 (3x3)
        % V_t{N} = zeros(numel(s));            % Alternative: zero time derivative / 替代方案：零时间导数
        V_t{N} = cost.phi_t(x{N}, x_star);    % Terminal value time derivative / 终端值时间导数
        % V_tt{N} = zeros(numel(s),numel(s));  % Alternative: zero time second derivative / 替代方案：零时间二阶导数
        V_tt{N} = cost.phi_tt(x{N}, x_star);  % Terminal value time second derivative / 终端值时间二阶导数
        % V_xt{N} = zeros(numel(x_star), numel(s)); % Alternative: zero cross derivative / 替代方案：零交叉导数
        V_xt{N} = cost.phi_xt(x{N}, x_star);  % Terminal value cross derivative (3x1) / 终端值交叉导数 (3x1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Compute defect vector for terminal-defect correction / 计算多重射击的缺陷向量
        d = x{N}- x_star;  % State defect at terminal time / 终端时间的状态缺陷
        % d = zeros(3,1);  % Alternative: zero defect / 替代方案：零缺陷
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Perform backwards pass / 执行反向传播
        for k = N-1:-1:1
           
            % Regularized derivatives (commented out) / 正则化导数（已注释）
            % V_xx1{k+1} = V_xx{k+1} + 0.01 * eye(size(V_xx{k+1}));
            % V_tt1{k+1} = V_tt{k+1} + 0.01 * eye(size(V_tt{k+1}));
            % V_xx1{k+1} = V_xx{k+1};
            % V_ss1{k+1} = V_ss{k+1};

            % Compute state-action value function derivatives / 计算状态动作值函数导数
            if k == N-1
                % Special case for last time step with defect correction / 带缺陷修正的最后时间步特殊情况
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                %3*1
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1} + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d;
                %1*1
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1}...
                    + dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * d + V_xt{k+1}' * d;
            %1*1
            else
                % Standard case for other time steps / 其他时间步的标准情况
                Q_x{k} = cost.L_x(x{k}, x_star, u{k}, t_f, dt) + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                %3*1
                Q_u{k} = cost.L_u(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_x{k+1};
                %1*1
                Q_t{k} = cost.L_t(x{k}, x_star, u{k}, t_f, dt) + V_t{k+1} + ...
                    dyn.gamma(x{k}, u{k}, t_f, dt)' * V_x{k+1};
            %1*1
            end

            % Q-function Hessian w.r.t. state / Q函数对状态的海塞矩阵
            Q_xx{k} = cost.L_xx(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.Phi(x{k}, u{k}, t_f, dt);
            %3*3
            % Q-function Hessian w.r.t. control / Q函数对控制的海塞矩阵
            Q_uu{k} = cost.L_uu(x{k}, u{k}, t_f, dt) + dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            %1*1
            % Q-function cross derivative w.r.t. state and control / Q函数对状态和控制的交叉导数
            Q_xu{k} = cost.L_xu(x{k}, u{k}, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.beta(x{k}, u{k}, t_f, dt);
            %3*1
            Q_ux{k} = Q_xu{k}';  % Transpose of Q_xu / Q_xu的转置
            %1*3
            % Q-function cross derivative w.r.t. state and time / Q函数对状态和时间的交叉导数
            Q_xt{k} = cost.L_xt(x{k}, x_star, u{k}, t_f, dt) + ...
                dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + dyn.Phi(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %3*1
            Q_tx{k} = Q_xt{k}';  % Transpose of Q_xt / Q_xt的转置
            %1*3
            % Q-function cross derivative w.r.t. control and time / Q函数对控制和时间的交叉导数
            Q_ut{k} = cost.L_ut(x{k}, u{k}, t_f, dt) + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.beta(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %1*1
            Q_tu{k} = Q_ut{k}';  % Transpose of Q_ut / Q_ut的转置
            %1*1
            % Q-function second derivative w.r.t. time / Q函数对时间的二阶导数
            Q_tt{k} = cost.L_tt(x{k}, u{k}, t_f, dt) + V_tt{k+1} + ...
                2 * dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xt{k+1} + ...
                dyn.gamma(x{k}, u{k}, t_f, dt)' * V_xx{k+1} * dyn.gamma(x{k}, u{k}, t_f, dt);
            %1*1

            % Compute the value function derivatives / 计算值函数导数
            [gains,Q_uu{k},numerical]=ddpack.solveControlSystem(Q_uu{k},-[Q_u{k},Q_ux{k},Q_ut{k}],numerical);
            k_t{k}=gains(:,1);K_t{k}=gains(:,2:1+numel(x_0));M_t{k}=gains(:,end);

            % Update value function derivatives using control gains / 使用控制增益更新值函数导数
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


    

        % Compute time horizon update using Newton's method / 使用牛顿法计算时间范围更新
        dtheta=ddpack.checkedTimeSolve(V_tt{1},-V_t{1});
        if isempty(costHistory)
            costHistory=J(i);
            meritWeight=10+abs(J(i))/((1-rho)*max(1,norm(x{N}-x_star)));
            meritHistory=NaN;
        end
        direction=struct('feedforward',{k_t},'feedback',{K_t},'time_gain',{M_t},'time_change',dtheta);
        geometry=struct('dimension',0,'centers',[],'radii',[],'gain',0);
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
    
    fprintf("finished PDDP, assembling results for post-processing...\n");
    % t = linspace(0.0, s, N);  % Alternative time vector / 替代时间向量
    % Assemble solution with all results / 用所有结果组装解
    sol = assemble_solution(x, u, t, J, E, M, Theta, Q_u, Q_uu, Q_ux, 0,d);
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
% Inputs / 输入参数
%
% x               : locally optimal state trajectory / 局部最优状态轨迹
% u               : locally optimal control sequence / 局部最优控制序列
% t               : discretized time stamps / 离散化时间戳
% J               : iteration history of cost function / 成本函数的迭代历史
% E               : iteration history of control energy / 控制能量的迭代历史
% M               : iteration history of merit function / 价值函数的迭代历史
% Theta           : iteration history of time horizon / 时间范围的迭代历史
% Q_u, Q_uu, Q_ux : derivatives of state action value function / 状态动作值函数的导数
% error           : zero if no error, nonzero else / 无错误时为零，否则非零
% d               : defect vector / 缺陷向量
%
% Outputs / 输出参数
%
% sol : solution structure / 解结构
function sol = assemble_solution(x, u, t, J, E, M, Theta, Q_u, Q_uu, Q_ux, error, d)

    % Solution structure / 解结构
    sol = struct;
    sol.error = error;        % Error flag / 错误标志
    sol.x = x;                % State trajectory / 状态轨迹
    sol.u = u(1:numel(x));    % Controls aligned with returned state nodes
    sol.t = t;                % Time vector / 时间向量
    sol.dt = t(2) - t(1);     % Time step / 时间步长
    sol.J = J;                % Cost history / 成本历史
    sol.E = E;                % Energy history / 能量历史
    sol.M = M;                % Merit function history / 价值函数历史
    sol.Theta = Theta;        % Time horizon history / 时间范围历史
    sol.Q_u = Q_u;            % Q-function control gradient / Q函数控制梯度
    sol.Q_uu = Q_uu;          % Q-function control Hessian / Q函数控制海塞矩阵
    sol.Q_ux = Q_ux;          % Q-function cross derivative / Q函数交叉导数
    sol.d = d;                % Defect vector / 缺陷向量
    % sol.dnorm = dnorm;      % Alternative: defect norm / 替代方案：缺陷范数
    % sol.log_d = log_d;      % Alternative: defect logging / 替代方案：缺陷记录

    return
end
% Update defect weight for terminal-defect correction / 为多重射击更新缺陷权重
function weight = UpdateDefectWeight(cost_first_order, dfectnorm, weight_prev,rho)

% If no defect, keep previous weight / 如果无缺陷，保持前一个权重
if dfectnorm == 0
    weight = weight_prev;
    return;
end

% Compute threshold for weight update / 计算权重更新的阈值
exp_change_abs = abs(cost_first_order);
thresh = 10 + exp_change_abs/((1-rho)*dfectnorm);

% Update weight based on threshold / 基于阈值更新权重
if weight_prev >= thresh
    weight = weight_prev;
else
    weight = thresh;
end

% Ensure minimum weight / 确保最小权重
weight = max(10, weight);

end

% Compute defect norm for terminal-defect correction / 为多重射击计算缺陷范数
function cost = ComputeDefectNorm(d, norm_id)
    norms = norm(d, norm_id); % Calculate the specified norm of the vector / 计算向量的指定范数
    cost = norm(norms, norm_id); % Calculate the specified norm again for all norm results / 对所有范数结果再次计算指定范数
end

% Proportional guidance for trajectory initialization / 用于轨迹初始化的比例制导
function [path, control] = Proportional_guidance(statr_point, end_point, tf, ~)
%UNTITLED2 Summary of this function is displayed here
%   Detailed description is displayed here


% Initialize velocity and position / 初始化速度和位置
vel = [0.0; 0.0; 0.0];  % velocity [v_x,v_y,q_dot] / 速度 [v_x,v_y,q_dot]
pos = [statr_point(1); statr_point(2); statr_point(3)];  % position [x,y,gamma] / 位置 [x,y,gamma]
q_dot = 0;  % angle velocity / 角速度
% path = [];  % Alternative path storage / 替代路径存储
% control = [];  % Alternative control storage / 替代控制存储

% Proportional gain / 比例增益
K = 5;

dt = 0.1;  % Time step / 时间步长

N = floor(tf ./ dt);  % Number of time steps / 时间步数

% Initialize output arrays / 初始化输出数组
path = cell(N, 1);
control = cell(N, 1);

% Generate trajectory using proportional guidance / 使用比例制导生成轨迹
for k = 1:N-1
    % Target and missile velocities / 目标和导弹速度
    V_Ty = 0;  % Target velocity in y / 目标y方向速度
    V_My = vel(2);  % Missile velocity in y / 导弹y方向速度
    V_Tx = 0;  % Target velocity in x / 目标x方向速度
    V_Mx = vel(1);  % Missile velocity in x / 导弹x方向速度

    % Target and missile positions / 目标和导弹位置
    Tx = end_point(1);  % Target x position / 目标x位置
    Mx = pos(1);        % Missile x position / 导弹x位置
    Ty = end_point(2);  % Target y position / 目标y位置
    My = pos(2);        % Missile y position / 导弹y位置
    
    % path = [path, pos];  % Alternative path storage / 替代路径存储
    % control = [control, K*q_dot];  % Alternative control storage / 替代控制存储
    path{k} = pos;  % Store current position / 存储当前位置
    control{k} = K * q_dot;  % Store current control / 存储当前控制
    
    % Proportional guidance law / 比例制导律
    q_dot = ( ( V_Ty - V_My )*( Tx - Mx ) - ( Ty - My )*( V_Tx - V_Mx ) ) / ( (Tx - Mx)^2 + (Ty - My)^2 );
    % Update position / 更新位置
    pos = pos + vel .* dt;
    % Update velocity based on heading / 基于航向更新速度
    vel(1) = 1.0 * cos(pos(3));
    vel(2) = 1.0 * sin(pos(3));
    vel(3) = K*q_dot;
end

% path = [path, pos];  % Alternative final position / 替代最终位置
% control = [control, K*q_dot];  % Alternative final control / 替代最终控制
path{N} = pos;  % Store final position / 存储最终位置
control{N} = K * q_dot;  % Store final control / 存储最终控制

end

