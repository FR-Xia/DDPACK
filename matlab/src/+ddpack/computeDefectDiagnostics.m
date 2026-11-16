function diagnostics = computeDefectDiagnostics(states, controls, time, problem, algorithm, raw)
%COMPUTEDEFECTDIAGNOSTICS Compare solver-consistent and RK4 propagation.
% The legacy solvers use zero-order-held controls and explicit Euler
% rollout. RK4 is reported separately as a continuous-model discretization
% check; it is not substituted for the method used by the optimizer.

if nargin < 5, algorithm = ''; end
if nargin < 6, raw = struct(); end
stateCount = size(states, 1);
stepCount = max(0, size(states, 2) - 1);
solverDefects = NaN(stateCount, stepCount);
rk4Defects = NaN(stateCount, stepCount);
completed = 0;
failureStep = NaN;
failureIdentifier = '';
failureMessage = '';

for index = 1:stepCount
    step = time(index + 1) - time(index);
    state = states(:, index);
    control = controls(:, index);
    try
        derivative = physicalDerivative(problem.dynamics, state, control);
        solverPrediction = state + step * derivative;
        rk4Prediction = rk4WithSubsteps( ...
            problem.dynamics, state, control, step, 4);
        if ~isreal(solverPrediction) || ~isreal(rk4Prediction) || ...
                any(~isfinite(solverPrediction)) || any(~isfinite(rk4Prediction))
            error('ddpack:diagnostics:NonfinitePrediction','Dynamics prediction is not finite and real.');
        end
        solverDefects(:, index) = states(:, index + 1) - solverPrediction;
        rk4Defects(:, index) = states(:, index + 1) - rk4Prediction;
        if any(~isfinite(solverDefects(:,index))) || any(~isfinite(rk4Defects(:,index)))
            error('ddpack:diagnostics:NonfiniteDefect','Dynamics defect is nonfinite.');
        end
        completed = completed + 1;
    catch exception
        failureStep = index;
        failureIdentifier = exception.identifier;
        failureMessage = exception.message;
        break
    end
end

scale = max(1, max(abs(states), [], 2));
relativeSolverDefects = abs(solverDefects) ./ scale;
relativeRk4Defects = abs(rk4Defects) ./ scale;
[solverMaximum, solverComponent, solverStep] = locateMaximum(solverDefects);
[rk4Maximum, rk4Component, rk4Step] = locateMaximum(rk4Defects);

reportedDefect = NaN;
if isfield(raw, 'd') && isnumeric(raw.d) && ~isempty(raw.d) && ...
        all(isfinite(raw.d(:)))
    reportedDefect = norm(raw.d(:), inf);
end

diagnostics = struct( ...
    'complete', stepCount > 0 && completed == stepCount, ...
    'evaluated_step_count', completed, 'expected_step_count', stepCount, ...
    'failure_step', failureStep, 'failure_identifier', failureIdentifier, ...
    'failure_message', failureMessage, ...
    'algorithm', char(algorithm), ...
    'solver_integration_method', 'explicit_euler_zoh', ...
    'independent_integration_method', 'rk4_zoh_four_substeps', ...
    'physical_time', time(:).', ...
    'physical_step_sizes', diff(time(:).'), ...
    'solver_reported_defect_norm', reportedDefect, ...
    'solver_consistent_defects', solverDefects, ...
    'solver_consistent_step_norms', max(abs(solverDefects), [], 1), ...
    'solver_consistent_relative_step_norms', max(relativeSolverDefects, [], 1), ...
    'solver_consistent_absolute_max', solverMaximum, ...
    'solver_consistent_relative_max', finiteMaximum(relativeSolverDefects), ...
    'solver_consistent_max_step', solverStep, ...
    'solver_consistent_max_component', solverComponent, ...
    'high_accuracy_defects', rk4Defects, ...
    'high_accuracy_step_norms', max(abs(rk4Defects), [], 1), ...
    'high_accuracy_relative_step_norms', max(relativeRk4Defects, [], 1), ...
    'high_accuracy_absolute_max', rk4Maximum, ...
    'high_accuracy_relative_max', finiteMaximum(relativeRk4Defects), ...
    'high_accuracy_max_step', rk4Step, ...
    'high_accuracy_max_component', rk4Component);
end

function derivative = physicalDerivative(dynamics, state, control)
try
    derivative = dynamics.F(state, control);
catch exception
    if ~any(strcmp(exception.identifier,{'MATLAB:minrhs','MATLAB:maxrhs', ...
            'MATLAB:TooManyInputs','MATLAB:notEnoughInputs','MATLAB:NotEnoughInputs'}))
        rethrow(exception);
    end
    derivative = dynamics.F(state, control, 1.0);
end
if ~isnumeric(derivative) || ~isreal(derivative) || ~isvector(derivative) || ...
        numel(derivative)~=numel(state) || any(~isfinite(derivative(:)))
    error('ddpack:diagnostics:InvalidDerivative','Dynamics must return one finite real derivative per state.');
end
derivative = derivative(:);
end

function state = rk4WithSubsteps(dynamics, state, control, step, count)
substep = step / count;
for substepIndex = 1:count %#ok<NASGU>
    k1 = physicalDerivative(dynamics, state, control);
    k2 = physicalDerivative(dynamics, state + 0.5 * substep * k1, control);
    k3 = physicalDerivative(dynamics, state + 0.5 * substep * k2, control);
    k4 = physicalDerivative(dynamics, state + substep * k3, control);
    state = state + substep * (k1 + 2*k2 + 2*k3 + k4) / 6;
end
end

function [value, component, step] = locateMaximum(values)
absoluteValues = abs(values);
if any(~isfinite(absoluteValues(:)))
    value = Inf; component = NaN; step = NaN;
    return
end
[value, linearIndex] = max(absoluteValues(:));
if isempty(value) || isinf(value)
    value = NaN; component = NaN; step = NaN;
else
    [component, step] = ind2sub(size(values), linearIndex);
end
end

function value = finiteMaximum(values)
if isempty(values)
    value = NaN;
elseif any(~isfinite(values(:)))
    value = Inf;
else
    value = max(values(:));
end
end
