% Abstract base class for defining cost functions in optimal control problems
% 最优控制问题中定义代价函数的抽象基类
classdef (Abstract) Cost
    methods (Abstract)
        % Abstract method: Calculate terminal state cost
        % 抽象方法：计算终端状态代价
        % Inputs:
        %   obj   - cost function object instance
        %   x_f   - final state vector
        %   x_bar - reference final state vector
        % Outputs:
        %   phi   - terminal state cost value
        % 输入参数：
        %   obj   - 代价函数对象实例
        %   x_f   - 最终状态向量
        %   x_bar - 参考最终状态向量
        % 输出参数：
        %   phi   - 终端状态代价值
        phi = phi(obj, x_f, x_bar);
        
        % Abstract method: Calculate first-order partial derivatives of terminal cost with respect to final state
        % 抽象方法：计算终端代价对最终状态的一阶偏导数
        % Inputs:
        %   obj    - cost function object instance
        %   x_f    - final state vector
        %   x_star - reference final state vector
        % Outputs:
        %   phi_x  - gradient of terminal cost with respect to final state
        % 输入参数：
        %   obj    - 代价函数对象实例
        %   x_f    - 最终状态向量
        %   x_star - 参考最终状态向量
        % 输出参数：
        %   phi_x  - 终端代价对最终状态的梯度
        phi_x = phi_x(obj, x_f, x_star);
        
        % Abstract method: Calculate second-order partial derivatives of terminal cost with respect to final state
        % 抽象方法：计算终端代价对最终状态的二阶偏导数
        % Inputs:
        %   obj    - cost function object instance
        %   x_f    - final state vector
        %   x_star - reference final state vector
        % Outputs:
        %   phi_xx - Hessian matrix of terminal cost with respect to final state
        % 输入参数：
        %   obj    - 代价函数对象实例
        %   x_f    - 最终状态向量
        %   x_star - 参考最终状态向量
        % 输出参数：
        %   phi_xx - 终端代价对最终状态的Hessian矩阵
        phi_xx = phi_xx(obj, x_f, x_star);
        
        % Abstract method: Calculate running cost at current time step
        % 抽象方法：计算当前时间步的运行代价
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L   - running cost value at current time step
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L   - 当前时间步的运行代价值
        L = L(obj, x, u, dt);
        
        % Abstract method: Calculate first-order partial derivatives of running cost with respect to state
        % 抽象方法：计算运行代价对状态的一阶偏导数
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_x - gradient of running cost with respect to state
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_x - 运行代价对状态的梯度
        L_x = L_x(obj, x, u, dt);
        
        % Abstract method: Calculate first-order partial derivatives of running cost with respect to control
        % 抽象方法：计算运行代价对控制的一阶偏导数
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_u - gradient of running cost with respect to control
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_u - 运行代价对控制的梯度
        L_u = L_u(obj, x, u, dt);
        
        % Abstract method: Calculate second-order partial derivatives of running cost with respect to state
        % 抽象方法：计算运行代价对状态的二阶偏导数
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_xx - Hessian matrix of running cost with respect to state
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_xx - 运行代价对状态的Hessian矩阵
        L_xx = L_xx(obj, x, u, dt);
        
        % Abstract method: Calculate second-order partial derivatives of running cost with respect to control
        % 抽象方法：计算运行代价对控制的二阶偏导数
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_uu - Hessian matrix of running cost with respect to control
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_uu - 运行代价对控制的Hessian矩阵
        L_uu = L_uu(obj, x, u, dt);
        
        % Abstract method: Calculate mixed second-order partial derivatives of running cost
        % 抽象方法：计算运行代价的混合二阶偏导数
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_xu - mixed Hessian matrix of running cost (state-control cross derivatives)
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_xu - 运行代价的混合Hessian矩阵（状态-控制交叉导数）
        L_xu = L_xu(obj, x, u, dt);
        
        % Abstract method: Calculate mixed second-order partial derivatives of running cost (transpose)
        % 抽象方法：计算运行代价的混合二阶偏导数（转置）
        % Inputs:
        %   obj - cost function object instance
        %   x   - state vector at current time step
        %   u   - control input vector at current time step
        %   dt  - time step size
        % Outputs:
        %   L_ux - mixed Hessian matrix of running cost (control-state cross derivatives)
        % 输入参数：
        %   obj - 代价函数对象实例
        %   x   - 当前时间步的状态向量
        %   u   - 当前时间步的控制输入向量
        %   dt  - 时间步长
        % 输出参数：
        %   L_ux - 运行代价的混合Hessian矩阵（控制-状态交叉导数）
        L_ux = L_ux(obj, x, u, dt);
        
        % Optional method: Calculate partial derivatives with respect to additional parameters
        % 可选方法：计算对额外参数的偏导数
        % (Currently commented out - may be implemented in future versions)
        % （当前已注释 - 可能在将来版本中实现）
        % L_s = L_s(obj, x, u, dt);
    end
end