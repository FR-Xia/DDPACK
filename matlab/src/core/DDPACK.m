classdef DDPACK < handle
    %DDPACK Backward-compatible object interface to the DDPACK solvers.
    %   OPTIMIZER = DDPACK() creates a finite-horizon regulation problem.
    %   Name-value arguments customize the
    %   algorithm, model, objective, horizon, constraints, and tolerances.
    %
    %   SOLUTION = OPTIMIZER.SOLVE() returns the normalized public solution
    %   schema. The solver-native structure is available as SOLUTION.raw.

    properties
        algorithm_name = 'free_time_pddp'
        legacy_algorithm_name = ''
        dimension = 2
        x0 = [1.2; 0]
        xf = [0; 0]
        tf = 15
        dt = 0.3
        num_iter = 80
        epsilon = 0.2
        alpha = 0.2
        rho = 0.05
        update_epsilon = 0.95
        u_max = 2
        constraints = {}
        obs_position = []
        obs_radius = []
        safety_gain = 0.5
        gamma_h = []
        waypoints = []
        montecarlo = 0
        freetime = false
        interval = 50
        v = 1
        m = 1
        g = 9.81
        s_p = 30
        Q_f = []
        Q_s = []
        R = []
        terminal_tolerance = 0.01
        constraint_tolerance = 1e-6
        safety_margin = 0
        dynamics = []
        cost = []
        solution = []

    end

    methods
        function obj = DDPACK(varargin)
            parser = inputParser;
            parser.FunctionName = 'DDPACK';
            registry = ddpack.algorithmRegistry();
            algorithms = {registry.algorithm_id};
            for index = 1:numel(registry)
                algorithms = [algorithms, registry(index).legacy_names]; %#ok<AGROW>
            end
            addParameter(parser, 'algorithm', obj.algorithm_name, ...
                @(x) (ischar(x) || (isstring(x) && isscalar(x))) && ...
                any(strcmpi(char(x), algorithms)));
            addParameter(parser, 'dimension', obj.dimension, ...
                @(x) isnumeric(x) && isscalar(x) && any(x == [2, 3]));
            addParameter(parser, 'x0', obj.x0, @isFiniteColumn);
            addParameter(parser, 'xf', obj.xf, @isFiniteColumn);
            addParameter(parser, 'tf', obj.tf, @isPositiveScalar);
            addParameter(parser, 'dt', obj.dt, @isPositiveScalar);
            addParameter(parser, 'num_iter', obj.num_iter, @isPositiveInteger);
            addParameter(parser, 'epsilon', obj.epsilon, @isPositiveScalar);
            addParameter(parser, 'alpha', obj.alpha, ...
                @(x) isPositiveScalar(x) && x <= 1);
            addParameter(parser, 'rho', obj.rho, @isPositiveScalar);
            addParameter(parser, 'update_epsilon', obj.update_epsilon, ...
                @(x) (islogical(x) && isscalar(x)) || isPositiveScalar(x));
            addParameter(parser, 'u_max', obj.u_max, @isPositiveVector);
            addParameter(parser, 'constraints', obj.constraints, ...
                @(x) isempty(x) || iscell(x) || isobject(x));
            addParameter(parser, 'obs_position', obj.obs_position, ...
                @(x) isnumeric(x) && ismatrix(x) && all(isfinite(x(:))));
            addParameter(parser, 'obs_radius', obj.obs_radius, ...
                @(x) isnumeric(x) && isvector(x) && all(isfinite(x(:))) && all(x(:) > 0));
            addParameter(parser, 'safety_gain', obj.safety_gain, @isPositiveScalar);
            addParameter(parser, 'gamma_h', obj.gamma_h, ...
                @(x) isempty(x) || isPositiveScalar(x));
            addParameter(parser, 'waypoints', obj.waypoints, ...
                @(x) isnumeric(x) && ismatrix(x) && all(isfinite(x(:))));
            addParameter(parser, 'montecarlo', obj.montecarlo, @isNonnegativeInteger);
            addParameter(parser, 'freetime', obj.freetime, ...
                @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
            addParameter(parser, 'interval', obj.interval, @isPositiveInteger);
            addParameter(parser, 'v', obj.v, @isPositiveScalar);
            addParameter(parser, 'm', obj.m, @isPositiveScalar);
            addParameter(parser, 'g', obj.g, @isPositiveScalar);
            addParameter(parser, 's_p', obj.s_p, @isPositiveScalar);
            addParameter(parser, 'Q_f', obj.Q_f, @isFiniteMatrixOrEmpty);
            addParameter(parser, 'Q_s', obj.Q_s, @isFiniteMatrixOrEmpty);
            addParameter(parser, 'R', obj.R, @isFiniteMatrixOrEmpty);
            addParameter(parser, 'terminal_tolerance', obj.terminal_tolerance, @isPositiveScalar);
            addParameter(parser, 'constraint_tolerance', obj.constraint_tolerance, @isNonnegativeScalar);
            addParameter(parser, 'safety_margin', obj.safety_margin, @isNonnegativeScalar);
            addParameter(parser, 'dynamics', obj.dynamics, @(x) isempty(x) || isobject(x));
            addParameter(parser, 'cost', obj.cost, @(x) isempty(x) || isobject(x));
            parse(parser, varargin{:});

            resultNames = fieldnames(parser.Results);
            for index = 1:numel(resultNames)
                name = resultNames{index};
                if strcmp(name, 'algorithm')
                    obj.algorithm_name = lower(char(parser.Results.(name)));
                else
                    obj.(name) = parser.Results.(name);
                end
            end
            if ~isempty(obj.gamma_h)
                warning('ddpack:DeprecatedProblemField', ...
                    'gamma_h is deprecated; use safety_gain.');
                obj.safety_gain = obj.gamma_h;
            end
            obj.validate_configuration();
            obj.setup_dynamics_and_cost();
        end

        function validate_configuration(obj)
            [identity, wasLegacy] = ddpack.resolveAlgorithm( ...
                obj.algorithm_name, 'Warn', true);
            algorithm = identity.algorithm_id;
            if wasLegacy, obj.legacy_algorithm_name = obj.algorithm_name; end
            if obj.dt >= obj.tf
                error('ddpack:DDPACK:InvalidTimeGrid', 'dt must be smaller than tf.');
            end
            if numel(obj.x0) ~= numel(obj.xf)
                error('ddpack:DDPACK:StateDimension', ...
                    'x0 and xf must contain the same number of states.');
            end
            if strcmp(algorithm, 'geometric_safety_pddp_2d') && obj.dimension ~= 2
                error('ddpack:DDPACK:AlgorithmDimension', ...
                    'geometric_safety_pddp_2d requires dimension=2.');
            end
            if strcmp(algorithm, 'geometric_safety_pddp_3d') && obj.dimension ~= 3
                error('ddpack:DDPACK:AlgorithmDimension', ...
                    'geometric_safety_pddp_3d requires dimension=3.');
            end
            if startsWith(algorithm, 'geometric_safety_pddp')
                if isempty(obj.obs_position) || isempty(obj.obs_radius)
                    error('ddpack:DDPACK:MissingObstacles', ...
                        'The geometric safety variants require obs_position and obs_radius.');
                end
                if size(obj.obs_position, 1) ~= obj.dimension || ...
                        size(obj.obs_position, 2) ~= numel(obj.obs_radius)
                    error('ddpack:DDPACK:ObstacleDimension', ...
                        'Obstacle centers and radii do not match dimension/count.');
                end
            end
        end

        function setup_dynamics_and_cost(obj)
            identity = ddpack.resolveAlgorithm(obj.algorithm_name, 'Warn', false);
            algorithm = identity.algorithm_id;
            if isempty(obj.dynamics)
                switch algorithm
                    case 'free_time_pddp'
                        if numel(obj.x0) == 2
                            obj.dynamics = PointMassDynamics([0, 1; -1, -0.4], [0; 0.2], [0; 0]);
                        elseif numel(obj.x0) == 3
                            obj.dynamics = PointMassDynamics(zeros(3), [0; 1; 0], [1; 0; 0]);
                        else
                            error('ddpack:DDPACK:MissingDynamics', ...
                                'Supply a dynamics object for non-default state dimensions.');
                        end
                    case 'terminal_defect_pddp'
                        if numel(obj.x0) ~= 3
                            error('ddpack:DDPACK:MissingDynamics', ...
                                'Supply a dynamics object for non-default state dimensions.');
                        end
                        obj.dynamics = PointMassDynamics(zeros(3), [0; 1; 0], [1; 0; 0]);
                    case 'waypoint_pddp'
                        obj.dynamics = Dynamics2D_c_pddp(obj.v, obj.s_p);
                    case 'geometric_safety_pddp_2d'
                        obj.dynamics = Dynamics2D_cbf_mspddp(obj.v);
                    case 'geometric_safety_pddp_3d'
                        obj.dynamics = Dynamics3D_cbf_mspddp(obj.m, obj.g, obj.s_p);
                    case 'penalty_free_time_ddp'
                        if numel(obj.x0) == 3
                            obj.dynamics = PointMassDynamics(zeros(3), [0; 1; 0], [1; 0; 0]);
                        else
                            obj.dynamics = Dynamics3D_fft_cddp(obj.m, obj.g, obj.s_p);
                        end
                end
            end
            if isempty(obj.cost)
                stateCount = numel(obj.x0);
                if isempty(obj.Q_f), obj.Q_f = 100 * eye(stateCount); end
                if isempty(obj.Q_s), obj.Q_s = eye(stateCount); end
                if isempty(obj.R), obj.R = 0.2; end
                if strcmp(algorithm, 'penalty_free_time_ddp')
                    obj.cost = QuadraticCost_fft(obj.Q_f, obj.Q_s, obj.R);
                else
                    obj.cost = QuadraticCost(obj.Q_f, obj.Q_s, obj.R);
                end
            end
        end

        function solution = solve(obj)
            problem = struct();
            problem.name = 'DDPACK_object_problem';
            problem.x0 = obj.x0;
            problem.xf = obj.xf;
            problem.tf = obj.tf;
            problem.dt = obj.dt;
            problem.dynamics = obj.dynamics;
            problem.cost = obj.cost;
            problem.u_max = obj.u_max;
            problem.constraints = obj.constraints;
            problem.waypoints = obj.waypoints;
            problem.montecarlo = obj.montecarlo;
            problem.freetime = obj.freetime;
            problem.interval = obj.interval;
            problem.obstacle_centers = obj.obs_position;
            problem.obstacle_radii = obj.obs_radius;
            problem.safety_gain = obj.safety_gain;
            problem.terminal_tolerance = obj.terminal_tolerance;
            problem.constraint_tolerance = obj.constraint_tolerance;
            problem.safety_margin = obj.safety_margin;
            options = ddpack.Options('Algorithm', obj.algorithm_name, ...
                'MaxIterations', obj.num_iter, 'Alpha', obj.alpha, ...
                'Epsilon', obj.epsilon, 'Rho', obj.rho, ...
                'UpdateEpsilon', obj.update_epsilon);
            solution = ddpack.solve(problem, options);
            if ~isempty(obj.legacy_algorithm_name)
                solution.legacy_algorithm_name = obj.legacy_algorithm_name;
            end
            obj.solution = solution;
        end

        function figureHandle = plot_solution(obj, varargin)
            if isempty(obj.solution)
                error('ddpack:DDPACK:NoSolution', 'Call solve before plot_solution.');
            end
            figureHandle = ddpack.plot(obj.solution, varargin{:});
        end

        function save_solution(obj, filename)
            if isempty(obj.solution)
                error('ddpack:DDPACK:NoSolution', 'Call solve before save_solution.');
            end
            ddpack.save(obj.solution, filename);
        end

        function solution = load_solution(obj, filename)
            solution = ddpack.load(filename);
            obj.solution = solution;
        end
    end

    methods (Static)
        function value = version()
            %VERSION Return the DDPACK semantic version.
            value = ddpack.version();
        end
    end
end

function value = isFiniteColumn(x)
value = isnumeric(x) && iscolumn(x) && ~isempty(x) && all(isfinite(x));
end

function value = isPositiveScalar(x)
value = isnumeric(x) && isscalar(x) && isreal(x) && isfinite(x) && x > 0;
end

function value = isPositiveVector(x)
value = isnumeric(x) && isvector(x) && ~isempty(x) && ...
    isreal(x) && all(isfinite(x(:))) && all(x(:) > 0);
end

function value = isNonnegativeScalar(x)
value = isnumeric(x) && isscalar(x) && isreal(x) && isfinite(x) && x >= 0;
end

function value = isPositiveInteger(x)
value = isPositiveScalar(x) && x == floor(x);
end

function value = isNonnegativeInteger(x)
value = isNonnegativeScalar(x) && x == floor(x);
end

function value = isFiniteMatrixOrEmpty(x)
value = isnumeric(x) && ismatrix(x) && all(isfinite(x(:)));
end
