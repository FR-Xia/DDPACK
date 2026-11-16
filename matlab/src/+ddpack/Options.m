classdef Options
    %OPTIONS Validated solver options for ddpack.solve.

    properties (SetAccess = private)
        Algorithm = 'free_time_pddp'
        MaxIterations = 80
        Alpha = 0.2
        Epsilon = 0.2
        Rho = 0.05
        UpdateEpsilon = 0.95
        QuickMode = false
    end

    methods
        function obj = Options(varargin)
            parser = inputParser;
            parser.FunctionName = 'ddpack.Options';
            registry = ddpack.algorithmRegistry();
            stable = registry(strcmp({registry.stable_or_experimental}, 'stable'));
            algorithms = {stable.algorithm_id};
            for index = 1:numel(stable)
                algorithms = [algorithms, stable(index).legacy_names]; %#ok<AGROW>
            end
            addParameter(parser, 'Algorithm', obj.Algorithm, ...
                @(x) (ischar(x) || (isstring(x) && isscalar(x))) && ...
                any(strcmpi(char(x), algorithms)));
            addParameter(parser, 'MaxIterations', obj.MaxIterations, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1 && x == floor(x));
            addParameter(parser, 'Alpha', obj.Alpha, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 && x <= 1);
            addParameter(parser, 'Epsilon', obj.Epsilon, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
            addParameter(parser, 'Rho', obj.Rho, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 && x < 1);
            addParameter(parser, 'UpdateEpsilon', obj.UpdateEpsilon, ...
                @(x) (islogical(x) && isscalar(x)) || ...
                (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
            addParameter(parser, 'QuickMode', obj.QuickMode, ...
                @(x) islogical(x) && isscalar(x));
            parse(parser, varargin{:});

            obj.Algorithm = lower(char(parser.Results.Algorithm));
            obj.MaxIterations = parser.Results.MaxIterations;
            obj.Alpha = parser.Results.Alpha;
            obj.Epsilon = parser.Results.Epsilon;
            obj.Rho = parser.Results.Rho;
            obj.UpdateEpsilon = parser.Results.UpdateEpsilon;
            obj.QuickMode = parser.Results.QuickMode;
        end
    end
end
