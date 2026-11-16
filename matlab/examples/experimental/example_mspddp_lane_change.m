function result = example_mspddp_lane_change(varargin)
%EXAMPLE_MSPDDP_LANE_CHANGE Optimize a fixed-time AGV lane change.
settings = parseSettings(varargin{:});
rng(7, 'twister');
problem = struct();
problem.name = 'agv_lane_change';
problem.x0 = [0; 0; 0];
problem.xf = [5; 1.2; 0];
problem.tf = 10;
problem.dt = 0.2;
problem.dynamics = PointMassDynamics(zeros(3), [0; 1; 0], [1; 0; 0]);
problem.cost = QuadraticCost(diag([200, 200, 40]), diag([1, 2, 0.2]), 0.2);
problem.u_max = 1.5;
problem.terminal_tolerance = 1e-3;
problem.constraint_tolerance = 1e-8;
options = ddpack.Options('Algorithm', 'mspddp', 'MaxIterations', 80, ...
    'Epsilon', 0.2, 'Rho', 0.05, 'UpdateEpsilon', 0.95, ...
    'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
straightLine = linspace(problem.x0(2), problem.xf(2), size(solution.states, 2));
pathDeviation = max(abs(solution.states(2, :) - straightLine));
assert(solution.success, solution.message);
assert(pathDeviation > 0.25, 'MSPDDP lane-change trajectory is effectively straight.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 7, 'quick_mode', settings.QuickMode, ...
    'path_deviation', pathDeviation, 'units', struct('position', 'm', 'time', 's'));
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
