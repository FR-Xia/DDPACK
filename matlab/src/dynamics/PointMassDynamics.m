% PointMassDynamics - Linear point-mass dynamics for optimal-control examples.
%
% Continuous model:
%   x_dot = A*x + B*u + drift
%
% The existing PDDP implementation passes a time-scaling parameter t and
% then integrates on a normalized grid. This class follows the same
% convention used by the existing Dynamics* classes.
classdef PointMassDynamics < Dynamics
    properties
        A
        B
        drift
    end

    methods
        function obj = PointMassDynamics(A, B, drift)
            obj.A = A;
            obj.B = B;
            if nargin < 3 || isempty(drift)
                drift = zeros(size(A, 1), 1);
            end
            obj.drift = drift;
        end

        function dxdt = F(obj, x, u, t)
            if nargin < 4
                t = 1.0;
            end
            dxdt = t * (obj.A * x + obj.B * u + obj.drift);
        end

        function Phi = Phi(obj, x, u, t, dt) %#ok<INUSD>
            if nargin < 5
                dt = t;
                t = 1.0;
            end
            Phi = eye(size(obj.A, 1)) + t * obj.A * dt;
        end

        function beta = beta(obj, x, u, t, dt) %#ok<INUSD>
            if nargin < 5
                dt = t;
                t = 1.0;
            end
            beta = t * obj.B * dt;
        end

        function Beta = Beta(obj, x, u, t, dt)
            if nargin < 5
                dt = t;
                t = 1.0;
            end
            Beta = obj.beta(x, u, t, dt);
        end

        function theta = theta(obj, x, u, t, dt) %#ok<INUSD>
            if nargin < 5
                dt = t;
            end
            theta = (obj.A * x + obj.B * u + obj.drift) * dt;
        end

        function gamma = gamma(obj, x, u, t, dt) %#ok<INUSD>
            gamma = (obj.A * x + obj.B * u + obj.drift) * dt;
        end
    end
end
