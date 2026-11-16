% Class for dynamics of inverted pendulum system that inherits from / 继承自抽象动力学类的
% abstract Dynamics class. / 倒立摆系统动力学类
classdef Dynamics3D_pddp < Dynamics
    properties % Class properties / 类属性
        m; % mass of vehicle / 飞行器质量
        g; % gravitational acceleration / 重力加速度
        s_p; % scale parameter for altitude scaling / 高度缩放的尺度参数
    end
    
    methods % Class methods / 类方法
        % Constructor / 构造函数
        function obj = Dynamics3D_pddp(m, g, scale_para)
            obj.m = m; % Set vehicle mass / 设置飞行器质量
            obj.g = g; % Set gravitational acceleration / 设置重力加速度
            obj.s_p = scale_para; % Set scale parameter / 设置尺度参数
        end
        
        % Equations of motion in state space representation / 状态空间表示的动力学方程
        function dxdt = F(obj, x, u, t)
            % State vector: x = [x, y, z, V, gamma, psi] / 状态向量：x = [x, y, z, V, gamma, psi]
            % Control vector: u = [u1, u2] / 控制向量：u = [u1, u2]
            % x, y, z: position coordinates / x, y, z: 位置坐标
            % V: velocity magnitude / V: 速度大小
            % gamma: flight path angle / gamma: 飞行路径角
            % psi: heading angle / psi: 航向角
            
            rho = 1.15579 - 1.058*10^(-4)*x(3)*obj.s_p + 3.725*10^(-9)*(x(3)*obj.s_p)^2 - 6*10^(-14)*(x(3)*obj.s_p)^3; % air density model / 空气密度模型
            S = 0.0324; % Reference area / 参考面积
            Cd = 0.0169; % Drag coefficient / 阻力系数
            D = 0.5 .* rho * x(4)^2 * S * Cd; % Drag force / 阻力
            
            x_dot = x(4) .* cos(x(5)).* cos(x(6)) / obj.s_p; % x-velocity / x方向速度
            y_dot = x(4) .* cos(x(5)).* sin(x(6)) / obj.s_p; % y-velocity / y方向速度
            z_dot = x(4) .* sin(x(5)) / obj.s_p; % z-velocity / z方向速度
            V_dot = -D ./ obj.m - obj.g .* sin(x(5)); % velocity magnitude rate / 速度大小变化率
            gama_dot = ( -u(2) - obj.g .* cos(x(5)) ) ./ x(4); % flight path angle rate / 飞行路径角变化率
            psi_dot = u(1) ./ ( x(4) .* cos(x(5)) ); % heading angle rate / 航向角变化率
                                       
            dxdt = t*[x_dot; y_dot; z_dot; V_dot; gama_dot; psi_dot]; % State derivative / 状态导数
        end
        
        % Linearized equations of motion (determined using symbolic math) / 线性化动力学方程（使用符号数学确定）
        function Phi = Phi(obj, x, u, t, dt)
            % Compute state transition matrix Phi = I + F_x * dt / 计算状态转移矩阵 Phi = I + F_x * dt
            % where F_x is the Jacobian of F with respect to state x / 其中 F_x 是 F 对状态 x 的雅可比矩阵
            
            rho = 1.15579 - 1.058*10^(-4)*x(3)*obj.s_p + 3.725*10^(-9)*(x(3)*obj.s_p)^2 - 6*10^(-14)*(x(3)*obj.s_p)^3; % air density model / 空气密度模型
            S = 0.0324; % Reference area / 参考面积
            Cd = 0.0169; % Drag coefficient / 阻力系数
            
            F_x = zeros(numel(x)); % Initialize Jacobian matrix / 初始化雅可比矩阵
            
            % Partial derivatives of x-dot with respect to state / x方向速度对状态的偏导数
            F_x(1,1) = 0.0; % dx_dot/dx / dx_dot/dx
            F_x(1,2) = 0.0; % dx_dot/dy / dx_dot/dy
            F_x(1,3) = 0.0; % dx_dot/dz / dx_dot/dz
            F_x(1,4) = cos(x(5)) .* cos(x(6)) / obj.s_p; % dx_dot/dV / dx_dot/dV
            F_x(1,5) = -x(4) .* sin(x(5)) .* cos(x(6)) / obj.s_p; % dx_dot/dgamma / dx_dot/dgamma
            F_x(1,6) = -x(4) .* cos(x(5)) .* sin(x(6)) / obj.s_p; % dx_dot/dpsi / dx_dot/dpsi
            
            % Partial derivatives of y-dot with respect to state / y方向速度对状态的偏导数
            F_x(2,1) = 0.0; % dy_dot/dx / dy_dot/dx
            F_x(2,2) = 0.0; % dy_dot/dy / dy_dot/dy
            F_x(2,3) = 0.0; % dy_dot/dz / dy_dot/dz
            F_x(2,4) = cos(x(5)) .* sin(x(6)) / obj.s_p; % dy_dot/dV / dy_dot/dV
            F_x(2,5) = -x(4) .* sin(x(5)) .* sin(x(6)) / obj.s_p; % dy_dot/dgamma / dy_dot/dgamma
            F_x(2,6) = x(4) .* cos(x(5)) .* cos(x(6)) / obj.s_p; % dy_dot/dpsi / dy_dot/dpsi
            
            % Partial derivatives of z-dot with respect to state / z方向速度对状态的偏导数
            F_x(3,1) = 0.0; % dz_dot/dx / dz_dot/dx
            F_x(3,2) = 0.0; % dz_dot/dy / dz_dot/dy
            F_x(3,3) = 0.0; % dz_dot/dz / dz_dot/dz
            F_x(3,4) = sin(x(5)) / obj.s_p; % dz_dot/dV / dz_dot/dV
            F_x(3,5) = x(4) .* cos(x(5)) / obj.s_p; % dz_dot/dgamma / dz_dot/dgamma
            F_x(3,6) = 0.0; % dz_dot/dpsi / dz_dot/dpsi
            
            % Partial derivatives of V-dot with respect to state / 速度大小变化率对状态的偏导数
            F_x(4,1) = 0.0; % dV_dot/dx / dV_dot/dx
            F_x(4,2) = 0.0; % dV_dot/dy / dV_dot/dy
            F_x(4,3) = 0.0; % dV_dot/dz / dV_dot/dz
            F_x(4,4) = -rho * x(4) * S * Cd / obj.m; % dV_dot/dV / dV_dot/dV
            F_x(4,5) = -obj.g .* cos(x(5)); % dV_dot/dgamma / dV_dot/dgamma
            F_x(4,6) = 0.0; % dV_dot/dpsi / dV_dot/dpsi
            
            % Partial derivatives of gamma-dot with respect to state / 飞行路径角变化率对状态的偏导数
            F_x(5,1) = 0.0; % dgamma_dot/dx / dgamma_dot/dx
            F_x(5,2) = 0.0; % dgamma_dot/dy / dgamma_dot/dy
            F_x(5,3) = 0.0; % dgamma_dot/dz / dgamma_dot/dz
            F_x(5,4) = ( u(2) + obj.g .* cos(x(5)) ) ./ (x(4)^2); % dgamma_dot/dV / dgamma_dot/dV
            F_x(5,5) = ( obj.g .* sin(x(5)) ) ./ (x(4)); % dgamma_dot/dgamma / dgamma_dot/dgamma
            F_x(5,6) = 0.0; % dgamma_dot/dpsi / dgamma_dot/dpsi
            
            % Partial derivatives of psi-dot with respect to state / 航向角变化率对状态的偏导数
            F_x(6,1) = 0.0; % dpsi_dot/dx / dpsi_dot/dx
            F_x(6,2) = 0.0; % dpsi_dot/dy / dpsi_dot/dy
            F_x(6,3) = 0.0; % dpsi_dot/dz / dpsi_dot/dz
            F_x(6,4) = ( -u(1) ) ./ ( cos(x(5)) .* x(4)^2 ); % dpsi_dot/dV / dpsi_dot/dV
            F_x(6,5) = ( u(1) ) .* sin(x(5)) ./ ( cos(x(5))^2 .* x(4) ); % dpsi_dot/dgamma / dpsi_dot/dgamma
            F_x(6,6) = 0.0; % dpsi_dot/dpsi / dpsi_dot/dpsi
            
            Phi = eye(numel(x)) + t * F_x .* dt; % State transition matrix / 状态转移矩阵
        end
        function Beta = Beta(obj, x, u, t, dt)
            % Compute control influence matrix Beta = F_u * dt / 计算控制影响矩阵 Beta = F_u * dt
            % where F_u is the Jacobian of F with respect to control u / 其中 F_u 是 F 对控制 u 的雅可比矩阵
            
            F_u = zeros(numel(x), numel(u)); % Initialize control Jacobian matrix / 初始化控制雅可比矩阵
            
            % Partial derivatives of x-dot with respect to control / x方向速度对控制的偏导数
            F_u(1,1) = 0.0; % dx_dot/du1 / dx_dot/du1
            F_u(1,2) = 0.0; % dx_dot/du2 / dx_dot/du2
            
            % Partial derivatives of y-dot with respect to control / y方向速度对控制的偏导数
            F_u(2,1) = 0.0; % dy_dot/du1 / dy_dot/du1
            F_u(2,2) = 0.0; % dy_dot/du2 / dy_dot/du2
            
            % Partial derivatives of z-dot with respect to control / z方向速度对控制的偏导数
            F_u(3,1) = 0.0; % dz_dot/du1 / dz_dot/du1
            F_u(3,2) = 0.0; % dz_dot/du2 / dz_dot/du2
            
            % Partial derivatives of V-dot with respect to control / 速度大小变化率对控制的偏导数
            F_u(4,1) = 0.0; % dV_dot/du1 / dV_dot/du1
            F_u(4,2) = 0.0; % dV_dot/du2 / dV_dot/du2
            
            % Partial derivatives of gamma-dot with respect to control / 飞行路径角变化率对控制的偏导数
            F_u(5,1) = 0.0; % dgamma_dot/du1 / dgamma_dot/du1
            F_u(5,2) = -1 ./ x(4); % dgamma_dot/du2 / dgamma_dot/du2
            
            % Partial derivatives of psi-dot with respect to control / 航向角变化率对控制的偏导数
            F_u(6,1) = 1 ./ ( x(4) .* cos(x(5)) ); % dpsi_dot/du1 / dpsi_dot/du1
            F_u(6,2) = 0.0; % dpsi_dot/du2 / dpsi_dot/du2
                 
            Beta = t * F_u .* dt; % Control influence matrix / 控制影响矩阵
        end
        function theta = theta(obj, x, u, t, dt)
            % Compute time scaling factor for dynamics / 计算动力学的时间缩放因子
            % This method scales the dynamics by time factor / 此方法按时间因子缩放动力学
            theta = obj.F(x, u, t) .* dt ./ t; % Scaled state derivative / 缩放的状态导数
        end
    end
end

