function [sol, sol_init] = c_pddp(varargin)
%C_PDDP Deprecated alias for WAYPOINT_PDDP.
warning('ddpack:DeprecatedAlgorithmName', ...
    ['c_pddp is a legacy algorithm name. Use waypoint_pddp. ' ...
    'The implementation uses waypoint penalties and segment-time parameters.']);
[sol, sol_init] = waypoint_pddp(varargin{:});
end
