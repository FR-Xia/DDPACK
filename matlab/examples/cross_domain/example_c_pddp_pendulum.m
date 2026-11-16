function result = example_c_pddp_pendulum(varargin)
%EXAMPLE_C_PDDP_PENDULUM Deprecated example alias.
warning('ddpack:DeprecatedAlgorithmName', ...
    'Use example_waypoint_pddp_pendulum.');
result = example_waypoint_pddp_pendulum(varargin{:});
end
