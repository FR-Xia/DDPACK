% Class for 2D dynamics system for Multiple Shooting Projected Differential Dynamic Programming (MSPDDP) / 用于多重射击投影微分动态规划(MSPDDP)的二维动力学系统类
% This class implements the dynamics for a 2D system used in MSPDDP algorithm / 该类实现了用于MSPDDP算法的二维系统动力学
% Obstacle avoidance (×), Waypoints (√) / 避障 (×), 航点 (√)
classdef Dynamics2D_mspddp < Dynamics 
    properties % Class properties / 类属性
        % m; % mass parameter (currently commented out) / 质量参数（当前已注释）
        v; % velocity parameter (constant speed for the system) / 速度参数（系统的恒定速度）
    end

    methods % Class methods / 类方法
        % Constructor - Initialize 2D MSPDDP dynamics / 构造函数 - 初始化二维MSPDDP动力学
        function obj = Dynamics2D_mspddp(v)
            % obj.m = m; % Set mass parameter (currently disabled) / 设置质量参数（当前已禁用）
            obj.v = v; % Set velocity parameter / 设置速度参数
        end

        % Equations of motion in state space representation / 状态空间表示的运动方程
        function dxdt = F(obj, x, u, t)
            % State vector: x = [X_position, Y_position, alpha_angle] / 状态向量：x = [X位置, Y位置, alpha角度]
            % Control input: u = angular velocity (alpha_dot) / 控制输入：u = 角速度 (alpha_dot)
            % t: time scaling factor / t: 时间缩放因子

            % Calculate X-velocity component based on current angle / 根据当前角度计算X方向速度分量
            X_dot =  obj.v .* cos(x(3));
            
            % Calculate Y-velocity component based on current angle / 根据当前角度计算Y方向速度分量
            Y_dot =  obj.v .* sin(x(3));
            
            % Angular velocity is directly controlled / 角速度直接由控制输入决定
            alpha_dot =  u; 
                                       
            % Return state derivatives scaled by time parameter / 返回按时间参数缩放的状态导数
            dxdt = t * [X_dot; Y_dot; alpha_dot];
        end
        
        % Linearized equations of motion (state transition matrix) / 线性化运动方程（状态转移矩阵）
        % Calculated using symbolic mathematics for partial derivatives / 使用符号数学计算偏导数
        function Phi = Phi(obj, x, ~, t, dt)
            % Compute state transition matrix Phi = I + t * F_x * dt / 计算状态转移矩阵 Phi = I + t * F_x * dt
            % where F_x is the Jacobian of F with respect to state x / 其中 F_x 是 F 对状态 x 的雅可比矩阵
            
            % Initialize Jacobian matrix for state derivatives w.r.t. state / 初始化状态导数对状态的雅可比矩阵
            F_x = zeros(numel(x), numel(x));
            
            % Partial derivative of X_dot w.r.t. X_position / X_dot对X位置的偏导数
            F_x(1,1) = 0.0;
            
            % Partial derivative of X_dot w.r.t. Y_position / X_dot对Y位置的偏导数
            F_x(1,2) = 0.0;
            
            % Partial derivative of X_dot w.r.t. alpha_angle / X_dot对alpha角度的偏导数
            F_x(1,3) = - obj.v .* sin(x(3));

            % Partial derivative of Y_dot w.r.t. X_position / Y_dot对X位置的偏导数
            F_x(2,1) = 0.0;
            
            % Partial derivative of Y_dot w.r.t. Y_position / Y_dot对Y位置的偏导数
            F_x(2,2) = 0.0;
            
            % Partial derivative of Y_dot w.r.t. alpha_angle / Y_dot对alpha角度的偏导数
            F_x(2,3) = obj.v .* cos(x(3));
            
            % Partial derivative of alpha_dot w.r.t. X_position / alpha_dot对X位置的偏导数
            F_x(3,1) = 0.0;
            
            % Partial derivative of alpha_dot w.r.t. Y_position / alpha_dot对Y位置的偏导数
            F_x(3,2) = 0.0;
            
            % Partial derivative of alpha_dot w.r.t. alpha_angle / alpha_dot对alpha角度的偏导数
            F_x(3,3) = 0.0;
            
            % Calculate state transition matrix using first-order approximation / 使用一阶近似计算状态转移矩阵
            Phi = eye(numel(x)) + t * F_x .* dt;
        end
        
        % Linearized equations of motion (control influence matrix) / 线性化运动方程（控制影响矩阵）
        function beta = beta(~, x, ~, t, dt)
            % Compute control influence matrix beta = t * F_u * dt / 计算控制影响矩阵 beta = t * F_u * dt
            % where F_u is the Jacobian of F with respect to control u / 其中 F_u 是 F 对控制 u 的雅可比矩阵
            
            % Initialize Jacobian matrix for state derivatives w.r.t. control / 初始化状态导数对控制的雅可比矩阵
            F_u = zeros(numel(x), 1);
            
            % Partial derivative of X_dot w.r.t. control input / X_dot对控制输入的偏导数
            F_u(1) = 0.0;
            
            % Partial derivative of Y_dot w.r.t. control input / Y_dot对控制输入的偏导数
            F_u(2) = 0.0;
            
            % Partial derivative of alpha_dot w.r.t. control input / alpha_dot对控制输入的偏导数
            F_u(3) = 1.0;
                 
            % Calculate control influence matrix / 计算控制影响矩阵
            beta = t * F_u .* dt;
        end
        
        % Time derivative matrix (for MSPDDP algorithm) / 时间导数矩阵（用于MSPDDP算法）
        function gamma = gamma(obj, x, u, ~, dt)
            % Compute time derivative of state vector / 计算状态向量的时间导数
            % This method computes the partial derivative of F with respect to time / 此方法计算 F 对时间的偏导数
            % Used for MSPDDP algorithm with time-varying dynamics / 用于具有时变动力学的MSPDDP算法
            
            % Initialize time derivative vector / 初始化时间导数向量
            F_t = zeros(numel(x), 1);
        
            % Time derivative of X-position dynamics / X位置动力学的时间导数
            F_t(1) = obj.v .* cos(x(3));
            
            % Time derivative of Y-position dynamics / Y位置动力学的时间导数
            F_t(2) = obj.v .* sin(x(3));
            
            % Time derivative of alpha dynamics / alpha动力学的时间导数
            F_t(3) = u;
        
            % Calculate time derivative matrix scaled by time step / 计算按时间步缩放的时间导数矩阵
            gamma = F_t .* dt;
        end
    end
end