% Abstract base class for dynamics systems / 动力学系统的抽象基类
% This class defines the interface that all dynamics classes must implement / 该类定义了所有动力学类必须实现的接口
% All concrete dynamics classes should inherit from this abstract class / 所有具体动力学类都应继承自此抽象类
classdef (Abstract) Dynamics
    methods % Abstract methods / 抽象方法
        % Equations of motion in state space representation / 状态空间表示的运动方程
        % This method should be implemented by concrete dynamics classes / 此方法应由具体动力学类实现
        % Inputs: / 输入参数：
        %   obj - dynamics object / 动力学对象
        %   x   - state vector / 状态向量
        %   u   - control input vector / 控制输入向量
        % Outputs: / 输出参数：
        %   dxdt - state derivative vector / 状态导数向量
        dxdt = F(obj, x, u)
        
        % Linearized equations of motion (state transition matrix) / 线性化运动方程（状态转移矩阵）
        % This method computes the Jacobian of F with respect to state x / 此方法计算 F 对状态 x 的雅可比矩阵
        % Inputs: / 输入参数：
        %   obj - dynamics object / 动力学对象
        %   x   - state vector / 状态向量
        %   u   - control input vector / 控制输入向量
        %   dt  - time step / 时间步长
        % Outputs: / 输出参数：
        %   Phi - state transition matrix / 状态转移矩阵
        Phi = Phi(obj, x, u, dt);
        
        % Linearized equations of motion (control influence matrix) / 线性化运动方程（控制影响矩阵）
        % This method computes the Jacobian of F with respect to control u / 此方法计算 F 对控制 u 的雅可比矩阵
        % Inputs: / 输入参数：
        %   obj - dynamics object / 动力学对象
        %   x   - state vector / 状态向量
        %   u   - control input vector / 控制输入向量
        %   dt  - time step / 时间步长
        % Outputs: / 输出参数：
        %   beta - control influence matrix / 控制影响矩阵
        beta = beta(obj, x, u, dt);
    end
end
