function result = example_mspddp_multiple_shooting_trajectory(varargin)
%EXAMPLE_MSPDDP_MULTIPLE_SHOOTING_TRAJECTORY Deprecated example alias.
warning('ddpack:DeprecatedAlgorithmName', ...
    ['Use example_terminal_defect_trajectory. The historical example name ' ...
    'does not denote a complete multiple-shooting implementation.']);
result = example_terminal_defect_trajectory(varargin{:});
end
