% Class for inverted pendulum cost function that inherits from abstract Cost class / 
% 继承自抽象Cost类的倒立摆代价函数类
% This cost function penalizes deviation from the upright position and control effort / 
% 该代价函数惩罚偏离直立位置和控制努力
classdef InvertedPendulumCost < Cost
    properties
        Q_f;    % Terminal cost weights / 终端代价权重矩阵
        Q_s;    % Running cost state weights / 运行代价状态权重矩阵
        R;      % Control cost weights / 控制代价权重矩阵
        x_ref;  % Reference state (upright position: [0, 0]) / 参考状态（直立位置：[0, 0]）
    end
    
    methods
        % Constructor / 构造函数
        function obj = InvertedPendulumCost(Q_f, Q_s, R, x_ref)
            obj.Q_f = Q_f;
            obj.Q_s = Q_s;
            obj.R = R;
            if nargin < 4
                obj.x_ref = [0; 0];  % Default: upright position / 默认：直立位置
            else
                obj.x_ref = x_ref;
            end
        end
        
        % Terminal state cost / 终端状态代价
        function phi = phi(obj, x_f, x_bar)
            % Quadratic terminal cost: 0.5 * (x_f - x_bar)' * Q_f * (x_f - x_bar) / 
            % 二次终端代价：0.5 * (x_f - x_bar)' * Q_f * (x_f - x_bar)
            phi = 0.5 * (x_f - x_bar)' * obj.Q_f * (x_f - x_bar);
        end
        
        % Terminal state cost derivatives / 终端状态代价导数
        function phi_x = phi_x(obj, x_f, x_star)
            % Gradient of terminal cost: Q_f * (x_f - x_star) / 终端代价梯度：Q_f * (x_f - x_star)
            phi_x = obj.Q_f * (x_f - x_star);
        end
        
        function phi_xx = phi_xx(obj, ~, ~)
            % Hessian of terminal cost: Q_f / 终端代价海塞矩阵：Q_f
            phi_xx = obj.Q_f;
        end
        
        function phi_t = phi_t(obj, ~, ~)
            % Time derivative of terminal cost (zero for time-invariant terminal cost) / 
            % 终端代价对时间的导数（时不变终端代价为零）
            phi_t = 0.0;
        end
        
        function phi_xt = phi_xt(obj, x_f, ~)
            % Cross derivative vector (zero for time-invariant terminal cost) / 
            % 交叉导数向量（时不变终端代价为零）
            phi_xt = zeros(numel(x_f), 1);
        end
        
        function phi_tt = phi_tt(obj, ~, ~)
            % Time second derivative of terminal cost (zero for time-invariant terminal cost) / 
            % 终端代价对时间的二阶导数（时不变终端代价为零）
            phi_tt = 0.0;
        end
        
        % Running cost / 运行代价
        function L = L(obj, x, x_star, u, t, dt)
            % Time-weighted running cost: t * 0.5 * [(x - x_ref)' * Q_s * (x - x_ref) + u' * R * u] * dt / 
            % 时间加权运行代价：t * 0.5 * [(x - x_ref)' * Q_s * (x - x_ref) + u' * R * u] * dt
            L = t * (0.5 * (x - obj.x_ref)' * obj.Q_s * (x - obj.x_ref) + 0.5 * u' * obj.R * u) * dt;
        end
        
        % Running cost derivatives / 运行代价导数
        function L_x = L_x(obj, x, x_star, ~, t, dt)
            % Gradient w.r.t. state: t * Q_s * (x - x_ref) * dt / 对状态的梯度：t * Q_s * (x - x_ref) * dt
            L_x = t * obj.Q_s * (x - obj.x_ref) * dt;
        end
        
        function L_u = L_u(obj, ~, u, t, dt)
            % Gradient w.r.t. control: t * R * u * dt / 对控制的梯度：t * R * u * dt
            L_u = t * obj.R * u * dt;
        end
        
        function L_t = L_t(obj, x, x_star, u, ~, dt)
            % Time derivative: 0.5 * [(x - x_ref)' * Q_s * (x - x_ref) + u' * R * u] * dt / 
            % 时间导数：0.5 * [(x - x_ref)' * Q_s * (x - x_ref) + u' * R * u] * dt
            L_t = (0.5 * (x - obj.x_ref)' * obj.Q_s * (x - obj.x_ref) + 0.5 * u' * obj.R * u) * dt;
        end
        
        function L_xx = L_xx(obj, ~, ~, ~, t, dt)
            % Hessian w.r.t. state: t * Q_s * dt / 对状态的海塞矩阵：t * Q_s * dt
            L_xx = t * obj.Q_s * dt;
        end
        
        function L_uu = L_uu(obj, ~, ~, t, dt)
            % Hessian w.r.t. control: t * R * dt / 对控制的海塞矩阵：t * R * dt
            if isscalar(obj.R)
                L_uu = t * obj.R * dt;
            else
                L_uu = t * obj.R * dt;
            end
        end
        
        function L_tt = L_tt(~, ~, ~, ~, ~)
            % Time second derivative (zero for linear time dependence) / 时间二阶导数（线性时间依赖为零）
            L_tt = 0.0;
        end
        
        function L_xu = L_xu(~, x, u, ~)
            % Mixed Hessian w.r.t. state and control: zero matrix / 对状态和控制的混合海塞矩阵：零矩阵
            if isscalar(u)
                L_xu = zeros(numel(x), 1);
            else
                L_xu = zeros(numel(x), numel(u));
            end
        end
        
        function L_ux = L_ux(~, x, u, ~)
            % Mixed Hessian w.r.t. control and state: zero matrix / 对控制和状态的混合海塞矩阵：零矩阵
            if isscalar(u)
                L_ux = zeros(1, numel(x));
            else
                L_ux = zeros(numel(u), numel(x));
            end
        end
        
        function L_xt = L_xt(obj, x, x_star, ~, ~, dt)
            % Cross derivative w.r.t. state and time: Q_s * (x - x_ref) * dt / 
            % 对状态和时间的交叉导数：Q_s * (x - x_ref) * dt
            L_xt = obj.Q_s * (x - obj.x_ref) * dt;
        end
        
        function L_ut = L_ut(obj, ~, u, ~, dt)
            % Cross derivative w.r.t. control and time: R * u * dt / 
            % 对控制和时间的交叉导数：R * u * dt
            if isscalar(obj.R)
                L_ut = obj.R * u * dt;
            else
                L_ut = obj.R * u * dt;
            end
        end
    end
end

