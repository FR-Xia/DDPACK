function result = example_free_time_pddp_vibration_regulation(varargin)
%EXAMPLE_FREE_TIME_PDDP_VIBRATION_REGULATION Suppress flexible-mount vibration.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(4, 'twister');
problem = ddpack.problems.linearRegulation();
options = ddpack.Options('Algorithm', 'free_time_pddp', 'MaxIterations', 80, ...
    'Alpha', 0.2, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
assert(solution.success, solution.message);
assert(solution.terminal_error < 0.01, ...
    'PDDP terminal error exceeds the physically selected 0.01 tolerance.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 4, 'quick_mode', settings.QuickMode, ...
    'terminal_tolerance', problem.terminal_tolerance, ...
    'units', struct('time', 's', 'displacement', 'm', 'velocity', 'm/s'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
