%% Class for 3D circular obstacle constraints / 3D圆形障碍物约束类
% This class implements circular obstacle avoidance constraints for 3D systems / 该类为3D系统实现圆形障碍物避障约束
% Obstacle avoidance (√), Waypoints (×) / 避障 (√), 航点 (×)
classdef CircleConstraints_3D < Constraints
    properties % Class properties / 类属性
        Center; % center of the circular obstacle (x, y coordinates) / 圆形障碍物的中心点 (x, y 坐标)
        r; % radius of the circular obstacle / 圆形障碍物的半径
    end
    
    methods % Class methods / 类方法
        % Constructor - Initialize circular obstacle constraints / 构造函数 - 初始化圆形障碍物约束
        function obj = CircleConstraints_3D(Center, r)
            obj.Center = Center; % Set obstacle center position / 设置障碍物中心位置
            obj.r = r; % Set obstacle radius / 设置障碍物半径
        end
        
        % Circle constraints function - Calculate constraint violation / 圆形约束函数 - 计算约束违反程度
        function G = G(obj, x, ~, dt)
            % Calculate squared distance from point to circle center minus radius squared / 计算点到圆心距离的平方减去半径的平方
            % This function returns a positive value when the point is outside the circle / 当点在圆外时，此函数返回正值
            % and a negative value when the point is inside the circle / 当点在圆内时，此函数返回负值
            % The constraint is satisfied when G >= 0 (outside or on the circle).
            G = ( (x(1:2, 1)' - obj.Center) * (x(1:2, 1)' - obj.Center)' - obj.r^2 ) .*dt ;
        end
        
        % Circle constraints derivatives with respect to state variables / 圆形约束对状态变量的偏导数
        function G_x = G_x(obj, x, ~, dt)
            % Calculate gradient of constraint function with respect to x and y positions / 计算约束函数对x和y位置的梯度
            % The gradient is used in optimization algorithms to find feasible directions / 梯度用于优化算法中寻找可行方向
            % Only x and y positions affect the constraint, other state variables have zero gradient / 只有x和y位置影响约束，其他状态变量的梯度为零
            G_x = [2 .* (x(1:2, 1)' - obj.Center), 0, 0, 0, 0] .* dt;
        end
        
        % Circle constraints derivatives with respect to control inputs / 圆形约束对控制输入的偏导数
        function G_u = G_u(~, ~, u, ~)
            % Circle constraints are independent of control inputs / 圆形约束与控制输入无关
            % This method returns zero gradient since position constraints don't depend on control / 此方法返回零梯度，因为位置约束不依赖于控制
            G_u = zeros(1, numel(u));
        end
    end
end

