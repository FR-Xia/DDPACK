% Class for 2D constrained pendulum dynamics system that inherits from abstract Dynamics class / 继承自抽象动力学类的二维约束摆动力学系统类
% This class implements the dynamics for a 2D pendulum system with constrained motion / 该类实现了具有约束运动的二维摆系统的动力学
% Obstacle avoidance (×), Waypoints (√) / 避障 (×), 航点 (√)
classdef Dynamics2D_c_pddp < Dynamics
    properties % Class properties / 类属性
        s_p; % pendulum length parameter (scaling factor for position) / 摆长参数（位置缩放因子）
        v; % velocity of pendulum (constant speed parameter) / 摆的速度（恒定速度参数）
    end
    
    methods % Class methods / 类方法
        % Constructor - Initialize 2D constrained pendulum dynamics / 构造函数 - 初始化二维约束摆动力学
        function obj = Dynamics2D_c_pddp(v, s_p)
            obj.s_p = s_p; % Set pendulum length parameter / 设置摆长参数
            obj.v = v; % Set pendulum velocity parameter / 设置摆的速度参数
        end
        
        % Equations of motion in state space representation / 状态空间表示的运动方程
        function dxdt = F(obj, x, u, t)
            % State vector: x = [x_position, y_position, theta_angle] / 状态向量：x = [x位置, y位置, theta角度]
            % Control input: u = angular velocity (theta_dot) / 控制输入：u = 角速度 (theta_dot)
            % t: time scaling factor / t: 时间缩放因子
            
            % Calculate x-velocity component based on current angle / 根据当前角度计算x方向速度分量
            x_dot = obj.v .* cos(x(3)) / obj.s_p;
            
            % Calculate y-velocity component based on current angle / 根据当前角度计算y方向速度分量
            y_dot = obj.v .* sin(x(3)) / obj.s_p;
            
            % Angular velocity is directly controlled / 角速度直接由控制输入决定
            theta_dot = u;
                                       
            % Return state derivatives scaled by time parameter / 返回按时间参数缩放的状态导数
            dxdt = t * [x_dot; y_dot; theta_dot];
        end
        
        % Linearized equations of motion (state transition matrix) / 线性化运动方程（状态转移矩阵）
        % Calculated using symbolic mathematics for partial derivatives / 使用符号数学计算偏导数
        function Phi = Phi(obj, x, ~, t, dt)
            % Compute state transition matrix Phi = I + t * F_x * dt / 计算状态转移矩阵 Phi = I + t * F_x * dt
            % where F_x is the Jacobian of F with respect to state x / 其中 F_x 是 F 对状态 x 的雅可比矩阵
            % This method is used for C-PDDP algorithm / 此方法用于C-PDDP算法
            
            % Initialize Jacobian matrix for state derivatives w.r.t. state / 初始化状态导数对状态的雅可比矩阵
            F_x = zeros(numel(x),numel(x));
            
            % Partial derivative of x_dot w.r.t. x_position / x_dot对x位置的偏导数
            F_x(1,1) = 0.0;
            
            % Partial derivative of x_dot w.r.t. y_position / x_dot对y位置的偏导数
            F_x(1,2) = 0.0;
            
            % Partial derivative of x_dot w.r.t. theta_angle / x_dot对theta角度的偏导数
            F_x(1,3) = -obj.v .* sin(x(3)) / obj.s_p;
            
            % Partial derivative of y_dot w.r.t. x_position / y_dot对x位置的偏导数
            F_x(2,1) = 0.0;
            
            % Partial derivative of y_dot w.r.t. y_position / y_dot对y位置的偏导数
            F_x(2,2) = 0.0;
            
            % Partial derivative of y_dot w.r.t. theta_angle / y_dot对theta角度的偏导数
            F_x(2,3) = obj.v .* cos(x(3)) / obj.s_p;
            
            % Partial derivative of theta_dot w.r.t. x_position / theta_dot对x位置的偏导数
            F_x(3,1) = 0.0;
            
            % Partial derivative of theta_dot w.r.t. y_position / theta_dot对y位置的偏导数
            F_x(3,2) = 0.0;
            
            % Partial derivative of theta_dot w.r.t. theta_angle / theta_dot对theta角度的偏导数
            F_x(3,3) = 0.0;
            
            % Calculate state transition matrix using first-order approximation / 使用一阶近似计算状态转移矩阵
            Phi = eye(numel(x)) + t * F_x .* dt;
        end
        
        % Linearized equations of motion (control influence matrix) / 线性化运动方程（控制影响矩阵）
        function beta = beta(obj, x, ~, t, dt)
            % Compute control influence matrix beta = t * F_u * dt / 计算控制影响矩阵 beta = t * F_u * dt
            % where F_u is the Jacobian of F with respect to control u / 其中 F_u 是 F 对控制 u 的雅可比矩阵
            % This method is used for C-PDDP algorithm / 此方法用于C-PDDP算法
            
            % Initialize Jacobian matrix for state derivatives w.r.t. control / 初始化状态导数对控制的雅可比矩阵
            F_u = zeros(numel(x), 1);
            
            % Partial derivative of x_dot w.r.t. control input / x_dot对控制输入的偏导数
            F_u(1) = 0.0;
            
            % Partial derivative of y_dot w.r.t. control input / y_dot对控制输入的偏导数
            F_u(2) = 0.0;
            
            % Partial derivative of theta_dot w.r.t. control input / theta_dot对控制输入的偏导数
            F_u(3) = 1.0;
                 
            % Calculate control influence matrix / 计算控制影响矩阵
            beta = t * F_u .* dt;
        end
        
        % Time derivative of state dynamics (for time-varying systems) / 状态动力学的时间导数（用于时变系统）
        function theta = theta(obj, x, u, ~, dt)
            % Compute time derivative of state vector / 计算状态向量的时间导数
            % This method computes the partial derivative of F with respect to time / 此方法计算 F 对时间的偏导数
            % Used for C-PDDP algorithm with time-varying dynamics / 用于具有时变动力学的C-PDDP算法
            
            % Initialize time derivative vector / 初始化时间导数向量
            F_t = zeros(numel(x), 1);
            
            % Time derivative of x-position dynamics / x位置动力学的时间导数
            F_t(1) = obj.v .* cos(x(3)) / obj.s_p;
            
            % Time derivative of y-position dynamics / y位置动力学的时间导数
            F_t(2) = obj.v .* sin(x(3)) / obj.s_p;
            
            % Time derivative of theta dynamics / theta动力学的时间导数
            F_t(3) = u;
                 
            % Calculate time derivative scaled by time step / 计算按时间步缩放的时间导数
            theta = F_t .* dt;
        end
    end
end

