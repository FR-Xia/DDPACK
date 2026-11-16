% Class for quadratic terminal and running costs that inherits from abstract Cost class / 
% 继承自抽象Cost类的二次终端和运行成本类，支持时间相关成本函数
classdef QuadraticCost < Cost
    properties
        Q_f; % terminal cost weights / 终端成本权重矩阵
        Q_s; % running cost state weights / 运行成本状态权重矩阵
        R; % control cost weights / 控制成本权重矩阵
    end
    
    methods
        % Constructor / 构造函数
        % Initialize the quadratic cost function with weight matrices / 使用权重矩阵初始化二次成本函数
        function obj = QuadraticCost(Q_f, Q_s, R)
            % Set terminal cost weight matrix / 设置终端成本权重矩阵
            obj.Q_f = Q_f;
            % Set running cost state weight matrix / 设置运行成本状态权重矩阵
            obj.Q_s = Q_s;
            % Set control cost weight matrix / 设置控制成本权重矩阵
            obj.R = R;
		end
        
        % Terminal state cost / 终端状态成本
        % Calculate the terminal cost based on final state deviation / 基于最终状态偏差计算终端成本
        function phi = phi(obj, x_f, x_bar)
            % Quadratic terminal cost: 0.5 * (x_f - x_bar)' * Q_f * (x_f - x_bar) / 
            % 二次终端成本：0.5 * (x_f - x_bar)' * Q_f * (x_f - x_bar)
            phi = 0.5 .* (x_f - x_bar).' * obj.Q_f * (x_f - x_bar);
        end
        
        % Terminal state cost derivatives / 终端状态成本导数
        % First derivative of terminal cost with respect to final state / 终端成本对最终状态的一阶导数
        function phi_x = phi_x(obj, x_f, x_star)
            % Gradient of terminal cost: Q_f * (x_f - x_star) / 终端成本梯度：Q_f * (x_f - x_star)
            phi_x = obj.Q_f * (x_f - x_star);
        end
        % First derivative of terminal cost with respect to time / 终端成本对时间的一阶导数
        function phi_t = phi_t(obj, x_f, x_star)
            % Time derivative of terminal cost (zero for time-invariant terminal cost) / 
            % 终端成本对时间的导数（时不变终端成本为零）
            phi_t = 0.0;
        end
        % Second derivative of terminal cost with respect to final state / 终端成本对最终状态的二阶导数
        function phi_xx = phi_xx(obj, ~, ~)
            % Hessian of terminal cost: Q_f / 终端成本海塞矩阵：Q_f
            phi_xx = obj.Q_f;
        end
        % Cross derivative of terminal cost w.r.t. final state and time / 终端成本对最终状态和时间的交叉导数
        function phi_xt = phi_xt(obj, x_f, ~)
            % Cross derivative vector (zero for time-invariant terminal cost) / 
            % 交叉导数向量（时不变终端成本为零）
            phi_xt = zeros(numel(x_f),1);
        end
        % Second derivative of terminal cost with respect to time / 终端成本对时间的二阶导数
        function phi_tt = phi_tt(obj, ~, ~)
            % Time second derivative of terminal cost (zero for time-invariant terminal cost) / 
            % 终端成本对时间的二阶导数（时不变终端成本为零）
            phi_tt = 0.0;
        end
        
        % Running cost / 运行成本
        % Calculate the time-weighted running cost for given state and control / 计算给定状态和控制的时间加权运行成本
        function L = L(obj, x, x_star, u, t, dt)
            % Time-weighted running cost: t * 0.5 * [(x - x_star)' * Q_s * (x - x_star) + u' * R * u] * dt / 
            % 时间加权运行成本：t * 0.5 * [(x - x_star)' * Q_s * (x - x_star) + u' * R * u] * dt
            L = t*(0.5 .* (x - x_star).' * obj.Q_s * (x - x_star) + 0.5 .* u.' * obj.R * u) .* dt;
        end
        
        % Running cost derivatives / 运行成本导数
        % First derivative of running cost with respect to state / 运行成本对状态的一阶导数
        function L_x = L_x(obj, x, x_star, ~, t, dt)
            % Gradient w.r.t. state: t * Q_s * (x - x_star) * dt / 对状态的梯度：t * Q_s * (x - x_star) * dt
            L_x = t*(obj.Q_s * (x - x_star)) .* dt;
        end
        % First derivative of running cost with respect to control / 运行成本对控制的一阶导数
        function L_u = L_u(obj, ~, u, t, dt)
            % Gradient w.r.t. control: t * R * u * dt / 对控制的梯度：t * R * u * dt
            L_u = t*(obj.R * u) .* dt;
        end
        % First derivative of running cost with respect to time / 运行成本对时间的一阶导数
        function L_t = L_t(obj, x, x_star, u, ~, dt)
            % Time derivative: 0.5 * [(x - x_star)' * Q_s * (x - x_star) + u' * R * u] * dt / 
            % 时间导数：0.5 * [(x - x_star)' * Q_s * (x - x_star) + u' * R * u] * dt
            L_t = (0.5 .* (x - x_star).' * obj.Q_s * (x - x_star) + 0.5 .* u.' * obj.R * u) .* dt;
        end
        % Second derivative of running cost with respect to state / 运行成本对状态的二阶导数
        function L_xx = L_xx(obj, ~, ~, ~, t, dt)
            % Hessian w.r.t. state: t * Q_s * dt / 对状态的海塞矩阵：t * Q_s * dt
            L_xx = t*obj.Q_s .* dt;
        end
        % Second derivative of running cost with respect to control / 运行成本对控制的二阶导数
        function L_uu = L_uu(obj, ~, ~, t, dt)
            % Hessian w.r.t. control: t * R * dt / 对控制的海塞矩阵：t * R * dt
            L_uu = t*obj.R .* dt;
        end
        % Second derivative of running cost with respect to time / 运行成本对时间的二阶导数
		function L_tt = L_tt(~, ~, ~, ~, ~)
            % Time second derivative (zero for linear time dependence) / 时间二阶导数（线性时间依赖为零）
            L_tt = 0.0;
        end
        % Cross derivative of running cost w.r.t. state and control / 运行成本对状态和控制的交叉导数
        function L_xu = L_xu(~, x, u, ~)
            % Cross derivative matrix (zero for quadratic cost) / 交叉导数矩阵（二次成本为零）
            L_xu = zeros(numel(x), numel(u));
        end
        % Cross derivative of running cost w.r.t. control and state / 运行成本对控制和状态的交叉导数
        function L_ux = L_ux(~, x, u, ~)
            % Cross derivative matrix (zero for quadratic cost) / 交叉导数矩阵（二次成本为零）
            L_ux = zeros(numel(u), numel(x));
        end
        % Cross derivative of running cost w.r.t. state and time / 运行成本对状态和时间的交叉导数
        function L_xt = L_xt(obj, x, x_star, ~, ~, dt)
            % Cross derivative w.r.t. state and time: Q_s * (x - x_star) * dt / 
            % 对状态和时间的交叉导数：Q_s * (x - x_star) * dt
            L_xt = (obj.Q_s * (x - x_star)) .* dt;
        end
        % Cross derivative of running cost w.r.t. control and time / 运行成本对控制和时间的交叉导数
        function L_ut = L_ut(obj, ~, u, ~, dt)
            % Cross derivative w.r.t. control and time: R * u * dt / 
            % 对控制和时间的交叉导数：R * u * dt
            L_ut = (obj.R * u) .* dt;
        end
    end
end

