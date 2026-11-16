function result = example_terminal_defect_trajectory(varargin)
%EXAMPLE_TERMINAL_DEFECT_TRAJECTORY Curved three-state planar maneuver.
settings = ddpack.exampleSettings(mfilename, varargin{:});
rng(22, 'twister');
problem = struct();
problem.name = 'terminal_defect_pddp_trajectory';
problem.x0 = [0; 0; 0];
problem.xf = [5; 1.2; 0];
problem.tf = 10;
problem.dt = 0.2;
problem.dynamics = PointMassDynamics(zeros(3), [0; 1; 0], [1; 0; 0]);
problem.cost = QuadraticCost(diag([200, 200, 40]), diag([1, 2, 0.2]), 0.2);
problem.u_max = 1.5;
problem.terminal_tolerance = 1e-3;
problem.constraint_tolerance = 1e-8;
problem.independent_dynamics_defect_tolerance = 0.01;
options = ddpack.Options('Algorithm', 'terminal_defect_pddp', 'MaxIterations', 80, ...
    'Epsilon', 0.2, 'Rho', 0.05, 'UpdateEpsilon', 0.95, ...
    'QuickMode', settings.QuickMode);
solverLog = evalc('solution = ddpack.solve(problem, options);');
boundaryIndices = unique(round(linspace(1, size(solution.states, 2), 5)));
diagnostics = solution.metadata.defect_diagnostics;
boundarySteps = max(1, boundaryIndices(2:end) - 1);
continuityDefect = max(diagnostics.solver_consistent_step_norms(boundarySteps));
straightLine = linspace(problem.x0(2), problem.xf(2), size(solution.states, 2));
pathDeviation = max(abs(solution.states(2, :) - straightLine));
solution.metadata.audit_boundary_indices = boundaryIndices;
solution.metadata.maximum_boundary_rollout_defect = continuityDefect;
assert(solution.success, solution.message);
assert(continuityDefect <= solution.metadata.problem.dynamics_defect_tolerance);
assert(pathDeviation > 0.25, 'Terminal-defect PDDP trajectory is effectively straight.');
figureHandle = ddpack.plot(solution, 'Visible', settings.Visible);
subplot(2, 2, 1); hold on;
plot(solution.states(1, boundaryIndices), solution.states(2, boundaryIndices), 'kd', ...
    'MarkerFaceColor', [1, 0.85, 0.15], 'MarkerSize', 7, ...
    'DisplayName', 'Audit boundaries');
legend('Location', 'best');
config = struct('example', mfilename, 'algorithm', options.Algorithm, ...
    'random_seed', 22, 'quick_mode', settings.QuickMode, ...
    'audit_boundary_indices', boundaryIndices, ...
    'boundary_definition', 'uniform rollout-audit boundaries', ...
    'maximum_boundary_rollout_defect', continuityDefect, ...
    'path_deviation', pathDeviation, ...
    'units', struct('position', 'm', 'time', 's'));
ddpack.writeExampleArtifacts(settings.OutputDir, solution, config, solverLog, figureHandle);
result = struct('name', mfilename, 'solution', solution, ...
    'boundary_rollout_defect', continuityDefect, 'output_dir', settings.OutputDir);
if ~settings.Visible, close(figureHandle); end
end
