classdef KinematicFlightDynamics < Dynamics
    %KINEMATICFLIGHTDYNAMICS Constant-speed 3-D flight-path model.
    % State: [x; y; z; speed; flight_path_angle; heading]
    % Control: [heading_rate; flight_path_angle_rate]

    methods
        function derivative = F(~, state, control, timeScale)
            if nargin < 4, timeScale = 1.0; end
            speed = state(4);
            gamma = state(5);
            heading = state(6);
            physicalDerivative = [ ...
                speed*cos(gamma)*cos(heading); ...
                speed*cos(gamma)*sin(heading); ...
                speed*sin(gamma); ...
                0; control(2); control(1)];
            derivative = timeScale * physicalDerivative;
        end

        function transition = Phi(~, state, ~, timeScale, step)
            speed = state(4);
            gamma = state(5);
            heading = state(6);
            jacobian = zeros(6);
            jacobian(1, 4) = cos(gamma)*cos(heading);
            jacobian(1, 5) = -speed*sin(gamma)*cos(heading);
            jacobian(1, 6) = -speed*cos(gamma)*sin(heading);
            jacobian(2, 4) = cos(gamma)*sin(heading);
            jacobian(2, 5) = -speed*sin(gamma)*sin(heading);
            jacobian(2, 6) = speed*cos(gamma)*cos(heading);
            jacobian(3, 4) = sin(gamma);
            jacobian(3, 5) = speed*cos(gamma);
            transition = eye(6) + timeScale * jacobian * step;
        end

        function influence = beta(~, ~, ~, timeScale, step)
            controlJacobian = zeros(6, 2);
            controlJacobian(6, 1) = 1;
            controlJacobian(5, 2) = 1;
            influence = timeScale * controlJacobian * step;
        end

        function sensitivity = gamma(obj, state, control, ~, step)
            sensitivity = obj.F(state, control, 1.0) * step;
        end
    end
end
