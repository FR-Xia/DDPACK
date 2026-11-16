% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause

% Class for dynamics of inverted pendulum system that inherits from 
% abstract Dynamics class.
classdef UAV2D_Dynamics < Dynamics
    properties
        v; % velosity of pendulum
        s_p; % scale factor
    end
    
    methods
        % Constructor
        function obj = UAV2D_Dynamics(v, s_p)
            obj.v = v;
            obj.s_p = s_p;
        end
        
        % Equations of motion in state space representation
        function dxdt = F(obj, x, u, t)
            x_dot = obj.v * cos(x(3)) / obj.s_p;
            y_dot = obj.v * sin(x(3)) / obj.s_p;
            theta_dot = u;
                                       
            dxdt = [x_dot; y_dot; theta_dot];
        end
        
        % Linearized equations of motion (determined using symbolic math)
        function Phi = Phi(obj, x, ~, t, dt)
            F_x = zeros(numel(x));
            
            F_x(1,1) = 0.0;
            F_x(1,2) = 0.0;
            F_x(1,3) = -obj.v * sin(x(3)) / obj.s_p;
            
            F_x(2,1) = 0.0;
            F_x(2,2) = 0.0;
            F_x(2,3) = obj.v * cos(x(3)) / obj.s_p;
            
            F_x(3,1) = 0.0;
            F_x(3,2) = 0.0;
            F_x(3,3) = 0.0;
            
            Phi = eye(numel(x)) + F_x .* dt;
        end
        function beta = Beta(obj, x, ~, t, dt)
            F_u = zeros(numel(x), 1);
            
            F_u(1) = 0.0;
            F_u(2) = 0.0;
            F_u(3) = 1.0;
                 
            beta = F_u .* dt;
        end
    end
end

