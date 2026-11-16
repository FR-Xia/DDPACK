function result = example_planar_point_mass_trajectory(varargin)
%EXAMPLE_PLANAR_POINT_MASS_TRAJECTORY Planar velocity-controlled transfer.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(21, 'twister');
problem = struct();
problem.name = 'planar_point_mass_trajectory';
problem.x0 = [0; 0];
problem.xf = [5; 2];
problem.tf = 5;
problem.dt = 0.05;
problem.dynamics = PointMassDynamics(zeros(2), [0; 1], [1; 0]);
problem.cost = QuadraticCost(diag([500, 500]), eye(2), 0.2);
problem.u_max = 2;
problem.freetime = 0;
problem.montecarlo = 0;
problem.waypoints = [];
problem.terminal_tolerance = 0.05;
problem.constraint_tolerance = 1e-8;
problem.independent_dynamics_defect_tolerance = 0.01;
options = ddpack.Options('Algorithm', 'waypoint_pddp', 'MaxIterations', 100, ...
    'Alpha', 0.4, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
assert(solution.success, solution.message);
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 21, 'quick_mode', settings.QuickMode, ...
    'model', 'x_dot=1, y_dot=u', ...
    'units', struct('position', 'm', 'time', 's', 'control', 'm/s'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
