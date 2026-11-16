% Class for inverted pendulum dynamics system / 倒立摆动力学系统类
% This class implements the dynamics for an inverted pendulum control problem / 
% 该类实现了倒立摆控制问题的动力学
% State: x = [theta, theta_dot] where theta is the angle from vertical / 
% 状态：x = [theta, theta_dot]，其中theta是偏离垂直方向的角度
% Control: u = torque applied to the pendulum / 控制：u = 施加在摆上的力矩
classdef InvertedPendulumDynamics < Dynamics 
    properties
        m;      % Mass of the pendulum (kg) / 摆的质量（千克）
        l;      % Length of the pendulum (m) / 摆的长度（米）
        g;      % Gravitational acceleration (m/s^2) / 重力加速度（米/秒²）
        b;      % Damping coefficient (optional, for friction) / 阻尼系数（可选，用于摩擦）
    end

    methods
        % Constructor - Initialize inverted pendulum dynamics / 构造函数 - 初始化倒立摆动力学
        function obj = InvertedPendulumDynamics(m, l, g, b)
            if nargin < 4
                b = 0.0;  % Default: no damping / 默认：无阻尼
            end
            obj.m = m;
            obj.l = l;
            obj.g = g;
            obj.b = b;
        end

        % Equations of motion in state space representation / 状态空间表示的运动方程
        function dxdt = F(obj, x, u, t)
            % State vector: x = [theta, theta_dot] / 状态向量：x = [theta, theta_dot]
            % Control input: u = torque / 控制输入：u = 力矩
            % t: time scaling factor / t: 时间缩放因子

            theta = x(1);      % Angle from vertical / 偏离垂直方向的角度
            theta_dot = x(2);  % Angular velocity / 角速度

            % Angular acceleration from dynamics / 动力学角加速度
            % θ_dot_dot = (m*g*l*sin(θ) + u - b*θ_dot) / (m*l^2)
            theta_dot_dot = (obj.m * obj.g * obj.l * sin(theta) + u - obj.b * theta_dot) / (obj.m * obj.l^2);

            % Return state derivatives scaled by time parameter / 返回按时间参数缩放的状态导数
            dxdt = t * [theta_dot; theta_dot_dot];
        end
        
        % Linearized equations of motion (state transition matrix) / 线性化运动方程（状态转移矩阵）
        function Phi = Phi(obj, x, ~, t, dt)
            % Compute state transition matrix Phi = I + t * F_x * dt / 计算状态转移矩阵 Phi = I + t * F_x * dt
            % where F_x is the Jacobian of F with respect to state x / 其中 F_x 是 F 对状态 x 的雅可比矩阵
            
            theta = x(1);
            
            % Initialize Jacobian matrix for state derivatives w.r.t. state / 初始化状态导数对状态的雅可比矩阵
            F_x = zeros(2, 2);
            
            % Partial derivative of theta_dot w.r.t. theta / theta_dot对theta的偏导数
            F_x(1, 1) = 0.0;
            
            % Partial derivative of theta_dot w.r.t. theta_dot / theta_dot对theta_dot的偏导数
            F_x(1, 2) = 1.0;
            
            % Partial derivative of theta_dot_dot w.r.t. theta / theta_dot_dot对theta的偏导数
            F_x(2, 1) = (obj.m * obj.g * obj.l * cos(theta)) / (obj.m * obj.l^2);
            
            % Partial derivative of theta_dot_dot w.r.t. theta_dot / theta_dot_dot对theta_dot的偏导数
            F_x(2, 2) = -obj.b / (obj.m * obj.l^2);
            
            % Calculate state transition matrix using first-order approximation / 使用一阶近似计算状态转移矩阵
            Phi = eye(2) + t * F_x * dt;
        end
        
        % Linearized equations of motion (control influence matrix) / 线性化运动方程（控制影响矩阵）
        function beta = beta(obj, x, u, t, dt)
            % Compute control influence matrix beta = t * F_u * dt / 计算控制影响矩阵 beta = t * F_u * dt
            % where F_u is the Jacobian of F with respect to control u / 其中 F_u 是 F 对控制 u 的雅可比矩阵
            
            % Initialize Jacobian matrix for state derivatives w.r.t. control / 初始化状态导数对控制的雅可比矩阵
            F_u = zeros(numel(x), 1);
            
            % Partial derivative of theta_dot w.r.t. control / theta_dot对控制的偏导数
            F_u(1) = 0.0;
            
            % Partial derivative of theta_dot_dot w.r.t. control / theta_dot_dot对控制的偏导数
            F_u(2) = 1.0 / (obj.m * obj.l^2);
            
            % Calculate control influence matrix / 计算控制影响矩阵
            beta = t * F_u * dt;
        end
        
        % Alias for beta method (uppercase) for compatibility with pddp / beta方法的别名（大写）以兼容pddp
        function Beta = Beta(obj, x, u, t, dt)
            % Compute control influence matrix Beta = t * F_u * dt / 计算控制影响矩阵 Beta = t * F_u * dt
            % This is an alias for the beta method to match pddp.m calling convention / 这是beta方法的别名，以匹配pddp.m的调用约定
            Beta = obj.beta(x, u, t, dt);
        end
        
        % Time scaling factor for dynamics / 动力学的时间缩放因子
        function theta = theta(obj, x, u, t, dt)
            % Compute time scaling factor for dynamics / 计算动力学的时间缩放因子
            % This method scales the dynamics by time factor / 此方法按时间因子缩放动力学
            theta = obj.F(x, u, t) .* dt ./ t;  % Scaled state derivative / 缩放的状态导数
        end
    end
end

