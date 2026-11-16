classdef StateLowerBoundConstraint < Constraints
    %STATELOWERBOUNDCONSTRAINT Enforce state(index) >= lowerBound.
    % The returned feasibility function follows DDPACK's G >= 0 contract.

    properties (SetAccess = immutable)
        StateIndex
        LowerBound
        StateDimension
    end

    methods
        function obj = StateLowerBoundConstraint(index, lowerBound, stateDimension)
            validateattributes(index, {'numeric'}, ...
                {'scalar', 'integer', '>=', 1, '<=', stateDimension});
            validateattributes(lowerBound, {'numeric'}, {'scalar', 'real', 'finite'});
            validateattributes(stateDimension, {'numeric'}, {'scalar', 'integer', 'positive'});
            obj.StateIndex = index;
            obj.LowerBound = lowerBound;
            obj.StateDimension = stateDimension;
        end

        function value = G(obj, state, ~, step)
            value = (state(obj.StateIndex) - obj.LowerBound) * step;
        end

        function gradient = G_x(obj, ~, ~, step)
            gradient = zeros(1, obj.StateDimension);
            gradient(obj.StateIndex) = step;
        end

        function gradient = G_u(~, ~, control, ~)
            gradient = zeros(1, numel(control));
        end
    end
end
