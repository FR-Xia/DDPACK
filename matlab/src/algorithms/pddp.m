function sol = pddp(varargin)
%PDDP Deprecated alias for FREE_TIME_PDDP.
warning('ddpack:DeprecatedAlgorithmName', ...
    ['pddp is a legacy algorithm name. Use free_time_pddp. ' ...
    'The implementation is a package-specific free-final-time PDDP variant.']);
sol = free_time_pddp(varargin{:});
end
