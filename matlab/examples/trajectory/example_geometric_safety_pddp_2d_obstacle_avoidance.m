function result = example_geometric_safety_pddp_2d_obstacle_avoidance(varargin)
%EXAMPLE_GEOMETRIC_SAFETY_PDDP_2D_OBSTACLE_AVOIDANCE Planar transfer.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(23, 'twister');
problem = struct();
problem.name = 'geometric_safety_pddp_2d_obstacle_avoidance';
problem.x0 = [0; 0; 0];
problem.xf = [5; 0; 0];
problem.tf = 10;
problem.dt = 0.2;
problem.dynamics = Dynamics2D_cbf_mspddp(1);
problem.cost = QuadraticCost(diag([200, 200, 40]), diag([1, 1, 0.2]), 0.2);
problem.u_max = 1.5;
problem.obstacle_centers = [2.5; 0.0];
problem.obstacle_radii = 0.22;
problem.safety_margin = 0.03;
problem.geometric_enforcement_margin = 0.12;
problem.safety_gain = 0.5;
problem.minimum_time = 5.0;
problem.maximum_time = 6.0;
problem.terminal_projection_tolerance = 0.005;
problem.terminal_projection_max_iterations = 30;
problem.geometric_refinement_iterations = 80;
problem.geometric_refinement_knots = 12;
problem.geometric_control_rate_weight = 0.02;
problem.geometric_refinement_free_time = true;
problem.terminal_tolerance = 0.005;
problem.constraint_tolerance = 1e-8;
problem.independent_dynamics_defect_tolerance = 0.02;
options = ddpack.Options('Algorithm', 'geometric_safety_pddp_2d', ...
    'MaxIterations', 80, 'Epsilon', 0.2, 'Rho', 0.05, ...
    'UpdateEpsilon', 0.95, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
straightClearances = abs(problem.obstacle_centers(2, :)) - ...
    problem.obstacle_radii - problem.safety_margin;
assert(any(straightClearances < 0), ...
    'No obstacle blocks the straight start-to-goal reference.');
assert(solution.success, solution.message);
assert(solution.terminal_error <= problem.terminal_tolerance);
assert(solution.min_obstacle_clearance >= -problem.constraint_tolerance);
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 23, 'quick_mode', settings.QuickMode, ...
    'straight_reference_clearances', straightClearances, ...
    'optimized_maneuver_time', solution.time(end), ...
    'terminal_projection', solution.raw.terminal_projection, ...
    'geometric_refinement', solution.raw.geometric_refinement, ...
    'units', struct('position', 'm', 'time', 's'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'straight_reference_clearances', straightClearances, ...
    'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
