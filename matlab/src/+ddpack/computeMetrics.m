function [metrics, diagnostics] = computeMetrics(states, controls, time, problem, costHistory, algorithm, raw)
%COMPUTEMETRICS Compute independent, solver-aware validation metrics.

if nargin < 6, algorithm = ''; end
if nargin < 7, raw = struct(); end

metrics = struct();
metrics.finite_values_pass = all(isfinite(states(:))) && ...
    all(isfinite(controls(:))) && all(isfinite(time(:))) && ...
    all(isfinite(costHistory(:)));
metrics.dimension_check_pass = size(states, 2) == numel(time) && ...
    size(controls, 2) == numel(time) && ...
    size(states, 1) == numel(problem.x0);
metrics.time_monotonic_pass = numel(time) > 1 && all(diff(time) > 0);
metrics.final_time = time(end)-time(1);
metrics.time_bound_violation = 0;
if isfield(problem,'minimum_time') && ~isempty(problem.minimum_time)
    metrics.time_bound_violation = max(metrics.time_bound_violation, problem.minimum_time-metrics.final_time);
end
if isfield(problem,'maximum_time') && ~isempty(problem.maximum_time)
    metrics.time_bound_violation = max(metrics.time_bound_violation, metrics.final_time-problem.maximum_time);
end
metrics.time_bounds_pass = metrics.time_bound_violation == 0;
metrics.initial_error = norm(states(:, 1) - problem.x0);
metrics.terminal_error = terminalError(states(:, end), problem);
metrics.maximum_control = max(abs(controls(:)));

limits = problem.u_max(:);
if isscalar(limits)
    limits = repmat(limits, size(controls, 1), 1);
elseif numel(limits) ~= size(controls, 1)
    error('ddpack:metrics:ControlDimension', ...
        'u_max must be scalar or match the number of control rows.');
end
controlViolation = abs(controls) - limits;
metrics.maximum_control_constraint_violation = ...
    max(0, max(controlViolation(:)));
metrics.maximum_state_constraint_violation = stateConstraintViolation( ...
    states, controls, problem.constraints);
metrics.max_constraint_violation = combineApplicableMax( ...
    metrics.maximum_control_constraint_violation, ...
    metrics.maximum_state_constraint_violation);

[nodeClearance, segmentClearance, clearanceDetails] = ...
    obstacleClearance(states, problem);
metrics.minimum_node_obstacle_clearance = nodeClearance;
metrics.minimum_segment_obstacle_clearance = segmentClearance;
metrics.minimum_obstacle_clearance = segmentClearance;
metrics.clearance_minimum_segment_index = clearanceDetails.segment_index;
metrics.clearance_minimum_obstacle_index = clearanceDetails.obstacle_index;
metrics.clearance_segment_fraction = clearanceDetails.segment_fraction;
metrics.clearance_check_method = clearanceDetails.method;
metrics.minimum_pairwise_distance = NaN;
audit = ddpack.verifyGeometricRollout(problem, controls, time);
metrics.analytic_zoh_clearance_lower_bound = audit.clearance_lower_bound;
metrics.analytic_zoh_terminal_error = audit.terminal_error;
metrics.analytic_zoh_sampled_clearance = audit.sampled_clearance;

diagnostics = ddpack.computeDefectDiagnostics( ...
    states, controls, time, problem, algorithm, raw);
metrics.absolute_dynamics_defect = diagnostics.solver_consistent_absolute_max;
metrics.relative_dynamics_defect = diagnostics.solver_consistent_relative_max;
metrics.high_accuracy_absolute_dynamics_defect = ...
    diagnostics.high_accuracy_absolute_max;
metrics.high_accuracy_relative_dynamics_defect = ...
    diagnostics.high_accuracy_relative_max;
metrics.dynamics_diagnostics_complete = diagnostics.complete;
metrics.dynamics_diagnostics_failure_identifier = diagnostics.failure_identifier;
metrics.dynamics_diagnostics_failure_message = diagnostics.failure_message;

% Backward-compatible aliases used by the 1.0 public result schema.
metrics.is_finite = metrics.finite_values_pass;
metrics.shape_valid = metrics.dimension_check_pass;
metrics.time_monotonic = metrics.time_monotonic_pass;
metrics.max_control = metrics.maximum_control;
metrics.dynamics_defect = metrics.absolute_dynamics_defect;
metrics.obstacle_clearance_applicable = ~isnan(metrics.minimum_obstacle_clearance);
metrics.min_obstacle_clearance = metrics.minimum_obstacle_clearance;

metrics.initial_objective = historyValue(raw, 'initial_cost', firstValue(costHistory));
metrics.returned_objective = historyValue( ...
    raw, 'returned_cost', lastValue(costHistory));
metrics.cost_decreased = isfinite(metrics.initial_objective) && ...
    isfinite(metrics.returned_objective) && ...
    metrics.returned_objective <= metrics.initial_objective*(1+1e-10);
metrics.line_search_accepted_steps=NaN;
metrics.line_search_rejected_trials=NaN;
if isfield(raw,'line_search')
    metrics.best_accepted_objective=metrics.returned_objective;
    metrics.accepted_iteration_count=raw.line_search.accepted_steps;
    trials=raw.line_search.trials;
    metrics.rejected_iteration_count=double(~isempty(trials) && trials(end,end)==0);
    metrics.line_search_accepted_steps=raw.line_search.accepted_steps;
    metrics.line_search_rejected_trials=raw.line_search.rejected_trials;
    metrics.acceptance_semantics='actual_merit_primary_with_separate_postprocessing';
elseif isfield(raw, 'best_accepted_cost')
    metrics.best_accepted_objective = raw.best_accepted_cost;
    metrics.accepted_iteration_count = historyValue( ...
        raw, 'accepted_count', numel(costHistory));
    metrics.rejected_iteration_count = historyValue(raw, 'rejected_count', NaN);
    metrics.acceptance_semantics = 'explicit_strict_best_accepted_history';
else
    % MSP/PDDP histories report the current accepted trajectory but do not
    % expose rejected trial trajectories as a separate solver-native series.
    metrics.best_accepted_objective = metrics.returned_objective;
    metrics.accepted_iteration_count = numel(costHistory);
    metrics.rejected_iteration_count = NaN;
    metrics.acceptance_semantics = ...
        'returned_accepted_iterate_no_separate_rejection_history';
end
metrics.termination_reason = textValue(raw, 'termination_reason', 'solver_completed');
metrics.convergence_status = textValue(raw, 'convergence_status', 'reported_result');
end

function value = terminalError(finalState, problem)
value = norm(finalState - problem.xf);
if ~isfield(problem, 'terminal_error_function') || ...
        isempty(problem.terminal_error_function)
    return
end
if ~isa(problem.terminal_error_function, 'function_handle')
    error('ddpack:metrics:TerminalErrorFunction', ...
        'problem.terminal_error_function must be a function handle.');
end
value = problem.terminal_error_function(finalState, problem.xf);
if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
        ~isfinite(value) || value < 0
    error('ddpack:metrics:TerminalErrorValue', ...
        'The custom terminal error must return one finite nonnegative scalar.');
end
end

function [nodeValue, segmentValue, details] = obstacleClearance(states, problem)
nodeValue = NaN;
segmentValue = NaN;
details = struct('method', 'not_applicable', 'segment_index', NaN, ...
    'obstacle_index', NaN, 'segment_fraction', NaN);
if isempty(problem.obstacle_centers)
    return
end
positionDimension = size(problem.obstacle_centers, 1);
if size(states, 1) < positionDimension
    return
end
nodeValue = realmax;
segmentValue = realmax;
details.method = 'exact_minimum_distance_to_linear_state_segments';
for obstacle = 1:size(problem.obstacle_centers, 2)
    center = problem.obstacle_centers(:, obstacle);
    radius = problem.obstacle_radii(obstacle) + problem.safety_margin;
    delta = states(1:positionDimension, :) - center;
    nodeValue = min(nodeValue, min(sqrt(sum(delta .^ 2, 1))) - radius);
    for segment = 1:(size(states, 2) - 1)
        startPoint = states(1:positionDimension, segment);
        endPoint = states(1:positionDimension, segment + 1);
        direction = endPoint - startPoint;
        denominator = direction.' * direction;
        if denominator <= eps
            fraction = 0;
        else
            fraction = max(0, min(1, ...
                ((center - startPoint).' * direction) / denominator));
        end
        closestPoint = startPoint + fraction * direction;
        clearance = norm(closestPoint - center) - radius;
        if clearance < segmentValue
            segmentValue = clearance;
            details.segment_index = segment;
            details.obstacle_index = obstacle;
            details.segment_fraction = fraction;
        end
    end
end
end

function value = stateConstraintViolation(states, controls, constraints)
value = NaN;
if isempty(constraints)
    return
end
value = 0;
widths = zeros(1,numel(constraints));
% G(x,u,dt)=dt*g(x,u) is the stage-weighted solver contract. A unit
% weight evaluates physical feasibility independently of the integration grid.
% Check every state, including the terminal state with its held control.
for index = 1:size(states, 2)
    controlIndex = min(index, size(states, 2) - 1);
    for constraintIndex = 1:numel(constraints)
        constraint = constraints(constraintIndex);
        if iscell(constraints), constraint = constraints{constraintIndex}; end
        try
            constraintValue = constraint.G( ...
                states(:, index), controls(:, controlIndex), 1);
        catch exception
            wrapped = MException('ddpack:metrics:ConstraintEvaluation', ...
                'Constraint %d failed at step %d: %s', ...
                constraintIndex, index, exception.message);
            throw(addCause(wrapped, exception));
        end
        if ~isnumeric(constraintValue) || ~isreal(constraintValue) || ...
                isempty(constraintValue) || ~isvector(constraintValue) || ...
                any(~isfinite(constraintValue(:)))
            error('ddpack:metrics:InvalidConstraintValue', ...
                'Constraint %d at node %d must return a nonempty finite real numeric vector.',constraintIndex,index);
        end
        if index == 1
            widths(constraintIndex) = numel(constraintValue);
        end
        if numel(constraintValue) ~= widths(constraintIndex)
            error('ddpack:metrics:ConstraintDimensions', ...
                'Constraint %d changed output size at node %d.',constraintIndex,index);
        end
        value = max(value, max(0, max(-constraintValue(:))));
    end
end
end

function value = combineApplicableMax(first, second)
applicable = [first, second];
applicable = applicable(~isnan(applicable));
if isempty(applicable), value = NaN; else, value = max(applicable); end
end

function value = historyValue(raw, fieldName, fallback)
value = fallback;
if isfield(raw, fieldName) && isnumeric(raw.(fieldName)) && ...
        isscalar(raw.(fieldName)) && isfinite(raw.(fieldName))
    value = raw.(fieldName);
end
end

function value = textValue(raw, fieldName, fallback)
value = fallback;
if isfield(raw, fieldName) && ...
        (ischar(raw.(fieldName)) || (isstring(raw.(fieldName)) && isscalar(raw.(fieldName))))
    value = char(raw.(fieldName));
end
end

function value = firstValue(history)
if isempty(history), value = NaN; else, value = history(1); end
end

function value = lastValue(history)
if isempty(history), value = NaN; else, value = history(end); end
end
