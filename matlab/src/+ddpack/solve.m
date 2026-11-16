function solution = solve(problem, options)
%SOLVE Solve a validated finite-horizon optimal-control problem.
%   SOLUTION = DDPACK.SOLVE(PROBLEM, OPTIONS) dispatches to an existing
%   DDPACK solver and returns a normalized result. PROBLEM is a struct with
%   x0, xf, tf, dt, dynamics, cost, and u_max fields. OPTIONS is a
%   ddpack.Options instance.

if nargin < 2 || isempty(options)
    options = ddpack.Options();
end
if ~isa(options, 'ddpack.Options')
    error('ddpack:solve:InvalidOptions', ...
        'options must be a ddpack.Options instance.');
end
[identity, wasLegacy] = ddpack.resolveAlgorithm( ...
    options.Algorithm, 'Warn', true);
algorithm = identity.algorithm_id;
problem = validateProblem(problem, algorithm);

maxIterations = options.MaxIterations;
if options.QuickMode
    maxIterations = min(maxIterations, 30);
end

started = tic;
switch algorithm
    case 'free_time_pddp'
        raw = free_time_pddp(problem.x0, problem.xf, problem.tf, problem.dt, ...
            problem.dynamics, problem.cost, problem.constraints, ...
            problem.u_max, maxIterations, options.Alpha);
    case 'waypoint_pddp'
        [raw, rawInitial] = waypoint_pddp(problem.montecarlo, problem.freetime, ...
            problem.x0, problem.waypoints, problem.xf, problem.tf, ...
            problem.dt, problem.dynamics, problem.cost, problem.u_max, ...
            maxIterations, options.Alpha);
        raw.initial_solution = rawInitial;
    case 'terminal_defect_pddp'
        raw = terminal_defect_pddp(problem.x0, problem.xf, problem.tf, problem.dt, ...
            problem.dynamics, problem.cost, problem.u_max, maxIterations, ...
            options.Epsilon, options.Rho, options.UpdateEpsilon);
    case 'geometric_safety_pddp_2d'
        raw = geometric_safety_pddp_2d(problem.x0, problem.xf, problem.tf, problem.dt, ...
            problem.dynamics, problem.cost, problem.u_max, maxIterations, ...
            options.Epsilon, options.Rho, options.UpdateEpsilon, ...
            problem.obstacle_centers, effectiveObstacleRadii(problem), ...
            problem.safety_gain, size(problem.obstacle_centers, 2) == 1, ...
            size(problem.obstacle_centers, 2), ...
            problem.minimum_time, problem.maximum_time);
        if ~isfield(raw,'error') || raw.error==0
            raw = ddpack.projectGeometricTerminal(raw, problem);
            raw = ddpack.refineGeometricTrajectory(raw, problem);
        end
    case 'geometric_safety_pddp_3d'
        raw = geometric_safety_pddp_3d(problem.x0, problem.xf, problem.tf, problem.dt, ...
            problem.dynamics, problem.cost, problem.u_max, maxIterations, ...
            options.Epsilon, options.Rho, options.UpdateEpsilon, ...
            problem.obstacle_centers, effectiveObstacleRadii(problem), ...
            problem.safety_gain, size(problem.obstacle_centers, 2) == 1, ...
            size(problem.obstacle_centers, 2), ...
            problem.minimum_time, problem.maximum_time);
        if ~isfield(raw,'error') || raw.error==0
            raw = ddpack.projectGeometricTerminal(raw, problem);
            raw = ddpack.refineGeometricTrajectory(raw, problem);
        end
    case 'penalty_free_time_ddp'
        raw = penalty_free_time_ddp(problem.x0, problem.xf, problem.tf, problem.interval, ...
            problem.dynamics, problem.cost, problem.constraints, ...
            problem.u_max, maxIterations, options.Alpha);
    otherwise
        error('ddpack:solve:UnsupportedAlgorithm', ...
            'Unsupported algorithm: %s', algorithm);
end

solution = ddpack.normalizeSolution(raw, algorithm, problem, toc(started));
if wasLegacy
    solution.legacy_algorithm_name = lower(char(options.Algorithm));
end
end

function problem = validateProblem(problem, algorithm)
if ~isstruct(problem) || ~isscalar(problem)
    error('ddpack:solve:InvalidProblem', 'problem must be a scalar struct.');
end
required = {'x0', 'xf', 'tf', 'dt', 'dynamics', 'cost', 'u_max'};
for index = 1:numel(required)
    if ~isfield(problem, required{index})
        error('ddpack:solve:MissingField', ...
            'problem.%s is required.', required{index});
    end
end
validateattributes(problem.x0, {'numeric'}, {'column', 'real', 'finite', 'nonempty'});
validateattributes(problem.xf, {'numeric'}, {'column', 'real', 'finite', 'numel', numel(problem.x0)});
validateattributes(problem.tf, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
validateattributes(problem.dt, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});
validateattributes(problem.u_max, {'numeric'}, {'vector', 'real', 'finite', 'positive'});
if problem.dt >= problem.tf
    error('ddpack:solve:InvalidTimeGrid', 'problem.dt must be smaller than problem.tf.');
end
if ~isobject(problem.dynamics) || ~ismethod(problem.dynamics, 'F')
    error('ddpack:solve:InvalidDynamics', ...
        'problem.dynamics must be an object implementing F.');
end
if ~isobject(problem.cost)
    error('ddpack:solve:InvalidCost', 'problem.cost must be a cost object.');
end
problem = applyLegacyProblemAliases(problem);

defaults = struct('constraints', {{}}, 'waypoints', [], 'montecarlo', 0, ...
    'freetime', 0, 'interval', max(2, round(problem.tf / problem.dt)), ...
    'obstacle_centers', [], 'obstacle_radii', [], 'safety_gain', 0.5, ...
    'terminal_tolerance', 0.1, 'constraint_tolerance', 1e-6, ...
    'initial_tolerance', 1e-10, 'dynamics_defect_tolerance', 1e-6, ...
    'independent_dynamics_defect_tolerance', Inf, ...
    'require_cost_decrease', true, ...
    'safety_margin', 0.0, 'geometric_enforcement_margin', 0.0, ...
    'minimum_time', [], 'maximum_time', [], ...
    'terminal_projection_tolerance', [], ...
    'terminal_projection_max_iterations', 20, ...
    'geometric_refinement_iterations', 0, 'geometric_refinement_knots', 12, ...
    'geometric_control_rate_weight', 0.02, 'geometric_refinement_free_time', false, ...
    'name', 'unnamed_problem');
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(problem, names{index})
        problem.(names{index}) = defaults.(names{index});
    end
end
validateConstraints(problem.constraints);
validateattributes(problem.geometric_refinement_iterations, {'numeric'}, ...
    {'scalar','integer','finite','nonnegative'});
validateattributes(problem.geometric_refinement_knots, {'numeric'}, ...
    {'scalar','integer','finite','>=',3});
validateattributes(problem.geometric_control_rate_weight, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
if ~islogical(problem.geometric_refinement_free_time) || ~isscalar(problem.geometric_refinement_free_time)
    error('ddpack:solve:RefinementFreeTime','Refinement free-time flag must be logical.');
end
if problem.geometric_refinement_iterations>0 && problem.geometric_refinement_free_time && ...
        (isempty(problem.minimum_time) || isempty(problem.maximum_time))
    error('ddpack:solve:RefinementTimeBounds','Free-time refinement requires both time bounds.');
end

validateattributes(problem.interval, {'numeric'}, ...
    {'scalar', 'integer', 'finite', '>=', 2});
validateattributes(problem.terminal_tolerance, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(problem.constraint_tolerance, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(problem.dynamics_defect_tolerance, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(problem.initial_tolerance, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(problem.independent_dynamics_defect_tolerance, {'numeric'}, ...
    {'scalar', 'real', 'nonnan', 'nonnegative'});
validateOptionalTimeBound(problem.minimum_time, 'minimum_time');
validateOptionalTimeBound(problem.maximum_time, 'maximum_time');
isGeometric = any(strcmp(algorithm,{'geometric_safety_pddp_2d','geometric_safety_pddp_3d'}));
if ~isGeometric && (~isempty(problem.minimum_time) || ~isempty(problem.maximum_time))
    error('ddpack:solve:UnsupportedTimeBounds', ...
        'minimum_time and maximum_time are supported only by the geometric algorithms.');
end
if isGeometric
    if isempty(problem.minimum_time), problem.minimum_time=max(problem.tf-20,5); end
    if isempty(problem.maximum_time), problem.maximum_time=problem.tf+7; end
end
if ~isempty(problem.terminal_projection_tolerance)
    validateattributes(problem.terminal_projection_tolerance, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
end
validateattributes(problem.terminal_projection_max_iterations, {'numeric'}, ...
    {'scalar', 'integer', 'finite', 'nonnegative'});
if ~isempty(problem.minimum_time) && ~isempty(problem.maximum_time) && ...
        problem.minimum_time >= problem.maximum_time
    error('ddpack:solve:TimeBounds', ...
        'problem.minimum_time must be smaller than problem.maximum_time.');
end
if ~islogical(problem.require_cost_decrease) || ~isscalar(problem.require_cost_decrease)
    error('ddpack:solve:CostDecreaseFlag', ...
        'problem.require_cost_decrease must be a logical scalar.');
end

if startsWith(algorithm, 'geometric_safety_pddp')
    expectedDimension = 2 + strcmp(algorithm, 'geometric_safety_pddp_3d');
    if isempty(problem.obstacle_centers) || isempty(problem.obstacle_radii)
        error('ddpack:solve:MissingObstacles', ...
            '%s requires obstacle centers and radii.', algorithm);
    end
    if size(problem.obstacle_centers, 1) ~= expectedDimension
        error('ddpack:solve:ObstacleDimension', ...
            'Obstacle centers must have %d rows for %s.', expectedDimension, algorithm);
    end
    if numel(problem.obstacle_radii) ~= size(problem.obstacle_centers, 2)
        error('ddpack:solve:ObstacleCount', ...
            'Each obstacle center must have one radius.');
    end
    validateattributes(problem.obstacle_radii, {'numeric'}, ...
        {'vector', 'real', 'finite', 'positive'});
    validateattributes(problem.safety_margin, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
    validateattributes(problem.geometric_enforcement_margin, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'});
end
end

function validateConstraints(constraints)
if isempty(constraints), return; end
if iscell(constraints)
    error('ddpack:solve:ConstraintContainer', ...
        ['Nonempty problem.constraints must be a constraint object or object ' ...
        'array, not a cell array.']);
end

if ~isobject(constraints)
    error('ddpack:solve:InvalidConstraints', ...
        'problem.constraints must be empty or a constraint object array.');
end
for index = 1:numel(constraints)
    for method = {'G', 'G_x', 'G_u'}
        if ~ismethod(constraints(index), method{1})
            error('ddpack:solve:ConstraintContract', ...
                ['Constraint %d must implement G, G_x, and G_u using the ' ...
                'G(x,u) >= 0 feasible convention.'], index);
        end
    end
end
end

function validateOptionalTimeBound(value, name)
if isempty(value), return; end
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename, ['problem.' name]);
end

function problem = applyLegacyProblemAliases(problem)
aliases = {'gamma_h', 'safety_gain'; ...
    'cbf_enforcement_margin', 'geometric_enforcement_margin'};
for index = 1:size(aliases, 1)
    legacy = aliases{index, 1}; current = aliases{index, 2};
    if ~isfield(problem, legacy), continue; end
    if isfield(problem, current) && ~isequaln(problem.(legacy), problem.(current))
        error('ddpack:solve:ConflictingLegacyField', ...
            'problem.%s conflicts with problem.%s.', legacy, current);
    end
    warning('ddpack:DeprecatedProblemField', ...
        'problem.%s is deprecated; use problem.%s.', legacy, current);
    problem.(current) = problem.(legacy);
end
end

function radii = effectiveObstacleRadii(problem)
% Inflate the geometric correction boundary for zero-order hold/Euler steps.
radii = problem.obstacle_radii + problem.safety_margin + ...
    problem.geometric_enforcement_margin;
end
