classdef (Abstract) Constraints
    %CONSTRAINTS Interface for stage constraints used by DDPACK solvers.
    %   G(x,u) >= 0 denotes feasibility throughout the public API.
    %   A conventional c(x,u) <= 0 constraint is represented as G = -c.

    methods (Abstract)
        % Return stage feasibility values using G(x,u) >= 0 feasible.
        G = G(obj, x, u, dt);

        % Return the Jacobian of G with respect to the state x.
        G_x = G_x(obj, x, u, dt);

        % Return the Jacobian of G with respect to the control u.
        G_u = G_u(obj, x, u, dt);
    end
end
