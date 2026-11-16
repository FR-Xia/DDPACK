function sol = mspddp(varargin)
%MSPDDP Deprecated alias for TERMINAL_DEFECT_PDDP.
warning('ddpack:DeprecatedAlgorithmName', ...
    ['mspddp is a legacy name and does not denote a complete ' ...
    'multiple-shooting implementation. Use terminal_defect_pddp.']);
sol = terminal_defect_pddp(varargin{:});
end
