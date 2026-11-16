% Class for dynamics of 3D vehicle system for CBF-MSPDDP that inherits from / 继承自抽象动力学类的
% abstract Dynamics class. / 用于CBF-MSPDDP的3D飞行器系统动力学类
classdef Dynamics3D_cbf_mspddp < Dynamics 
    properties % Class properties / 类属性
        m; % mass of vehicle / 飞行器质量
        g; % gravitational acceleration / 重力加速度
        s_p; % scale parameter for altitude scaling / 高度缩放的尺度参数
    end

    methods % Class methods / 类方法
        % Constructor / 构造函数
        function obj = Dynamics3D_cbf_mspddp(m,g,s_p)
            obj.m = m; % Set vehicle mass / 设置飞行器质量
            obj.g = g; % Set gravitational acceleration / 设置重力加速度
            obj.s_p = s_p; % Set scale parameter / 设置尺度参数
        end

        % 3D state equation / 3D状态方程
        function dxdt = F(obj, x, u, t)
            % State vector: x = [x, y, z, V, gamma, psi] / 状态向量：x = [x, y, z, V, gamma, psi]
            % Control vector: u = [u1, u2] / 控制向量：u = [u1, u2]
            % x, y, z: position coordinates / x, y, z: 位置坐标
            % V: velocity magnitude / V: 速度大小
            % gamma: flight path angle / gamma: 飞行路径角
            % psi: heading angle / psi: 航向角
            % t: time scaling factor / t: 时间缩放因子
            
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
                                       
            dxdt = t * [x_dot; y_dot; z_dot; V_dot; gama_dot; psi_dot]; % Scaled state derivative / 缩放的状态导数
        end
        
        % Linearized motion equation / 线性化运动方程
        function Phi = Phi(obj, x, u, t, dt)
            % Compute state transition matrix Phi = I + t * F_x * dt / 计算状态转移矩阵 Phi = I + t * F_x * dt
            % where F_x is the Jacobian of F with respect to state x / 其中 F_x 是 F 对状态 x 的雅可比矩阵
            % This method is used for CBF-MSPDDP algorithm / 此方法用于CBF-MSPDDP算法
            
            rho = 1.15579 - 1.058*10^(-4)*x(3)*obj.s_p + 3.725*10^(-9)*(x(3)*obj.s_p)^2 - 6*10^(-14)*(x(3)*obj.s_p)^3; % air density model / 空气密度模型
            S = 0.0324; % Reference area / 参考面积
            Cd = 0.0169; % Drag coefficient / 阻力系数

            F_x = zeros(numel(x), numel(x)); % Initialize Jacobian matrix / 初始化雅可比矩阵
            
            % Partial derivatives of x-dot with respect to state / x方向速度对状态的偏导数
            F_x(1,4) = cos(x(5)) .* cos(x(6)) / obj.s_p; % dx_dot/dV / dx_dot/dV
            F_x(1,5) = -x(4) .* sin(x(5)) .* cos(x(6)) / obj.s_p; % dx_dot/dgamma / dx_dot/dgamma
            F_x(1,6) = -x(4) .* cos(x(5)) .* sin(x(6)) / obj.s_p; % dx_dot/dpsi / dx_dot/dpsi

            % Partial derivatives of y-dot with respect to state / y方向速度对状态的偏导数
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
            
            % The rest remains zero / 其余元素保持为零
            Phi = eye(numel(x)) + t * F_x .* dt; % State transition matrix / 状态转移矩阵
        end
        
        % Effect of control input on state / 控制输入对状态的影响
        function beta = beta(~, x, u, t, dt)
            % Compute control influence matrix beta = t * F_u * dt / 计算控制影响矩阵 beta = t * F_u * dt
            % where F_u is the Jacobian of F with respect to control u / 其中 F_u 是 F 对控制 u 的雅可比矩阵
            % This method is used for CBF-MSPDDP algorithm / 此方法用于CBF-MSPDDP算法
            
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
                 
            beta = t * F_u .* dt; % Control influence matrix / 控制影响矩阵
        end
        
        % Rate of change of state with respect to time / 状态对时间的变化率
        function gamma = gamma(obj, x, u, ~, dt)
            % Compute time derivative of state vector / 计算状态向量的时间导数
            % This method computes the partial derivative of F with respect to time / 此方法计算 F 对时间的偏导数
            % Used for CBF-MSPDDP algorithm with time-varying dynamics / 用于具有时变动力学的CBF-MSPDDP算法
            
            rho = 1.15579 - 1.058*10^(-4)*x(3)*obj.s_p + 3.725*10^(-9)*(x(3)*obj.s_p)^2 - 6*10^(-14)*(x(3)*obj.s_p)^3; % air density model / 空气密度模型
            S = 0.0324; % Reference area / 参考面积
            Cd = 0.0169; % Drag coefficient / 阻力系数
            D = 0.5 .* rho * x(4)^2 * S * Cd; % Drag force / 阻力

            F_t = zeros(numel(x), 1); % Initialize time derivative vector / 初始化时间导数向量
        
            % Linear velocity components / 线速度分量
            F_t(1) = x(4) .* cos(x(5)).* cos(x(6)) / obj.s_p; % x-velocity time derivative / x方向速度时间导数
            F_t(2) = x(4) .* cos(x(5)).* sin(x(6)) / obj.s_p; % y-velocity time derivative / y方向速度时间导数
            F_t(3) = x(4) .* sin(x(5)) / obj.s_p; % z-velocity time derivative / z方向速度时间导数
            
            % Angular velocity components / 角速度分量
            F_t(4) = -D ./ obj.m - obj.g .* sin(x(5)); % velocity magnitude time derivative / 速度大小时间导数
            F_t(5) = ( -u(2) - obj.g .* cos(x(5)) ) ./ x(4); % flight path angle time derivative / 飞行路径角时间导数
            F_t(6) = u(1) ./ ( x(4) .* cos(x(5)) ); % heading angle time derivative / 航向角时间导数
        
            gamma = F_t .* dt; % Scaled time derivative / 缩放的时间导数
        end
    end
end