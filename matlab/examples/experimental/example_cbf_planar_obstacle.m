function result = example_cbf_planar_obstacle(varargin)
%EXAMPLE_CBF_PLANAR_OBSTACLE Route a delivery robot around a blocked path.
settings = parseSettings(varargin{:});
rng(11, 'twister');
problem = struct();
problem.name = 'hospital_delivery_obstacle';
problem.x0 = [0; 0; 0];
problem.xf = [5; 0; 0];
problem.tf = 10;
problem.dt = 0.2;
problem.dynamics = Dynamics2D_cbf_mspddp(1);
problem.cost = QuadraticCost(diag([200, 200, 40]), diag([1, 1, 0.2]), 0.2);
problem.u_max = 1.5;
problem.obstacle_centers = [2.5; 0.10];
problem.obstacle_radii = 0.12;
problem.safety_margin = 0;
problem.gamma_h = 0.5;
problem.terminal_tolerance = 0.35;
problem.constraint_tolerance = 1e-8;
options = ddpack.Options('Algorithm', 'cbf_mspddp_2d', ...
    'MaxIterations', 80, 'Epsilon', 0.2, 'Rho', 0.05, ...
    'UpdateEpsilon', 0.95, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
straightClearance = abs(problem.obstacle_centers(2)) - problem.obstacle_radii;
pathDeviation = max(abs(solution.states(2, :)));
assert(straightClearance < 0, 'The straight reference does not intersect the obstacle.');
assert(solution.success, solution.message);
assert(solution.min_obstacle_clearance >= -problem.constraint_tolerance, ...
    'The optimized trajectory intersects the obstacle.');
assert(pathDeviation > 0.15, 'The optimized trajectory does not visibly avoid the obstacle.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 11, 'quick_mode', settings.QuickMode, ...
    'straight_clearance', straightClearance, 'path_deviation', pathDeviation, ...
    'units', struct('position', 'm', 'time', 's'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'path_deviation', pathDeviation, 'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end

function settings = parseSettings(varargin)
parser = inputParser;
addParameter(parser, 'OutputDir', fullfile(tempdir, mfilename), @(x) ischar(x) || isstring(x));
addParameter(parser, 'Visible', true, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'QuickMode', false, @(x) islogical(x) && isscalar(x));
parse(parser, varargin{:}); settings = parser.Results;
settings.OutputDir = char(settings.OutputDir);
end
