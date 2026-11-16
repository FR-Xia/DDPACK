function result = example_geometric_safety_pddp_3d_obstacle_avoidance(varargin)
%EXAMPLE_GEOMETRIC_SAFETY_PDDP_3D_OBSTACLE_AVOIDANCE Spatial transfer.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(24, 'twister');
targetPosition = [5; 2; 1];
heading = atan2(targetPosition(2), targetPosition(1));
gamma = atan2(targetPosition(3), norm(targetPosition(1:2)));
speed = norm(targetPosition) / 5;
problem = struct();
problem.name = 'geometric_safety_pddp_3d_obstacle_avoidance';
problem.x0 = [0; 0; 0; speed; gamma; heading];
problem.xf = [targetPosition; speed; gamma; heading];
problem.tf = 5;
problem.dt = 0.1;
problem.dynamics = KinematicFlightDynamics();
problem.cost = QuadraticCost(diag([1000, 1000, 1000, 20, 100, 100]), ...
    diag([0.05, 0.05, 0.05, 0.01, 0.05, 0.05]), 0.2*eye(2));
problem.u_max = [1.5; 1.5];
problem.obstacle_centers = targetPosition * (24/49);
problem.obstacle_radii = 0.18;
problem.safety_margin = 0.04;
problem.geometric_enforcement_margin = 0.15;
problem.safety_gain = 0.2;
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
options = ddpack.Options('Algorithm', 'geometric_safety_pddp_3d', ...
    'MaxIterations', 80, 'Epsilon', 0.2, 'Rho', 0.05, ...
    'UpdateEpsilon', 0.95, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
lineDirection = problem.xf(1:3) - problem.x0(1:3);
centerOffset = problem.obstacle_centers - problem.x0(1:3);
straightDistance = norm(cross(centerOffset, lineDirection)) / norm(lineDirection);
straightClearance = straightDistance - problem.obstacle_radii - problem.safety_margin;
assert(straightClearance < 0, ...
    'The spherical obstacle does not block the straight 3-D reference.');
assert(solution.success, solution.message);
assert(solution.terminal_error <= problem.terminal_tolerance);
assert(solution.min_obstacle_clearance >= -problem.constraint_tolerance);
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 24, 'quick_mode', settings.QuickMode, ...
    'straight_reference_clearance', straightClearance, ...
    'geometric_enforcement_margin', problem.geometric_enforcement_margin, ...
    'optimized_maneuver_time', solution.time(end), ...
    'terminal_projection', solution.raw.terminal_projection, ...
    'geometric_refinement', solution.raw.geometric_refinement, ...
    'units', struct('position', 'm', 'time', 's', 'angle', 'rad'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'straight_reference_clearance', straightClearance, ...
    'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
