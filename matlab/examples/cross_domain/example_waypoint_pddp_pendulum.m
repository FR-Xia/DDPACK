function result = example_waypoint_pddp_pendulum(varargin)
%EXAMPLE_WAYPOINT_PDDP_PENDULUM Swing an inverted pendulum to upright balance.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(13, 'twister');
problem = struct();
problem.name = 'inverted_pendulum_swingup';
problem.x0 = [pi; 0];
problem.xf = [0; 0];
problem.tf = 5;
problem.dt = 0.05;
problem.dynamics = InvertedPendulumDynamics(1, 1, 9.81, 0.1);
problem.cost = InvertedPendulumCost(diag([100, 10]), diag([1, 0.1]), 0.1, problem.xf);
problem.u_max = 10;
problem.freetime = false;
problem.montecarlo = 0;
problem.terminal_tolerance = 1e-2;
problem.constraint_tolerance = 1e-8;
problem.terminal_error_function = @periodicTerminalError;
options = ddpack.Options('Algorithm', 'waypoint_pddp', 'MaxIterations', 120, ...
    'Alpha', 0.4, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
assert(solution.success, solution.message);
assert(solution.terminal_error < problem.terminal_tolerance, ...
    'Waypoint-PDDP pendulum terminal error exceeds 0.01.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 13, 'quick_mode', settings.QuickMode, ...
    'terminal_tolerance', problem.terminal_tolerance, ...
    'units', struct('angle', 'rad', 'angular_velocity', 'rad/s', 'torque', 'N m'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, 'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end

function value = periodicTerminalError(state, target)
angleError = mod(state(1) - target(1) + pi, 2*pi) - pi;
value = norm([angleError; state(2) - target(2)]);
end
