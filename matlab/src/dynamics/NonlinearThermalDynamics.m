classdef NonlinearThermalDynamics < Dynamics
    %NONLINEARTHERMALDYNAMICS Scalar nonlinear thermal-process model.
    % The state is normalized temperature deviation and the control is a
    % signed heater/cooler command; neither variable is spatial.

    methods
        function derivative = F(~, state, control, varargin)
            scale = NonlinearThermalDynamics.timeScale(varargin{:});
            derivative = scale * (-0.25 * state - 0.03 * state.^3 + 0.60 * control);
        end

        function transition = Phi(~, state, ~, varargin)
            [scale, step] = NonlinearThermalDynamics.scaleAndStep(varargin{:});
            transition = 1 + scale * (-0.25 - 0.09 * state.^2) * step;
        end

        function influence = beta(~, ~, ~, varargin)
            [scale, step] = NonlinearThermalDynamics.scaleAndStep(varargin{:});
            influence = scale * 0.60 * step;
        end

        function influence = Beta(obj, state, control, varargin)
            influence = obj.beta(state, control, varargin{:});
        end

        function derivative = theta(~, state, control, varargin)
            [~, step] = NonlinearThermalDynamics.scaleAndStep(varargin{:});
            derivative = (-0.25 * state - 0.03 * state.^3 + 0.60 * control) * step;
        end

        function derivative = gamma(obj, state, control, varargin)
            derivative = obj.theta(state, control, varargin{:});
        end
    end

    methods (Static, Access = private)
        function scale = timeScale(varargin)
            if isempty(varargin), scale = 1; else, scale = varargin{1}; end
        end

        function [scale, step] = scaleAndStep(varargin)
            if numel(varargin) == 1
                scale = 1; step = varargin{1};
            elseif numel(varargin) >= 2
                scale = varargin{1}; step = varargin{2};
            else
                scale = 1; step = 1;
            end
        end
    end
end
