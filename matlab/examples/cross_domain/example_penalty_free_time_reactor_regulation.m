function result = example_penalty_free_time_reactor_regulation(varargin)
%EXAMPLE_PENALTY_FREE_TIME_REACTOR_REGULATION Regulate a reactor model.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(17, 'twister');
problem = struct();
problem.name = 'reactor_concentration_regulation';
problem.x0 = 4;
problem.xf = 0;
problem.tf = 15;
problem.dt = 0.3;
problem.interval = 50;
problem.dynamics = PointMassDynamics(-0.30, 0.02, 0);
problem.cost = QuadraticCost_fft(200, 2, 2);
problem.u_max = 1;
problem.constraints = {};
problem.terminal_tolerance = 0.04;
problem.constraint_tolerance = 1e-8;
problem.independent_dynamics_defect_tolerance = 0.02;
options = ddpack.Options('Algorithm', 'penalty_free_time_ddp', 'MaxIterations', 80, ...
    'Alpha', 0.25, 'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
tail = abs(solution.states(1, max(1, end-10):end));
assert(solution.success, solution.message);
assert(all(diff(tail) <= 1e-4), ...
    'Accepted reactor trajectory grows near the terminal time.');
assert(solution.accepted_cost_history(end) < solution.accepted_cost_history(1));
assert(all(diff(solution.accepted_cost_history) < 0), ...
    'Accepted costs must be strict best-so-far improvements.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 17, 'quick_mode', settings.QuickMode, ...
    'cost_history_semantics', ...
        'trial costs and strict best-so-far accepted nominal costs', ...
    'returned_iteration', solution.raw.returned_iteration, ...
    'units', struct('time', 's', 'concentration_error', 'normalized'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
