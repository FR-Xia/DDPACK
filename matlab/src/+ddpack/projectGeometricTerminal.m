function raw = projectGeometricTerminal(raw, problem)
%PROJECTGEOMETRICTERMINAL Feasibility-preserving terminal projection.
%   The optional projection is a single-shooting control correction after
%   the geometry-aware PDDP pass.  A damped least-squares control update is
%   accepted only when it reduces terminal error and keeps every piecewise
%   linear trajectory segment outside the inflated obstacle boundary.

if isempty(problem.terminal_projection_tolerance) || ...
        problem.terminal_projection_max_iterations == 0
    return;
end
if ~isstruct(raw) || ~isfield(raw, 'x') || ~isfield(raw, 'u')
    return;
end

states = trajectoryMatrix(raw.x);
controls = trajectoryMatrix(raw.u);
nodeCount = size(states, 2);
if nodeCount < 2 || size(controls, 2) < nodeCount - 1
    return;
end
controls = controls(:, 1:nodeCount-1);
timeHorizon = problem.tf;
if isfield(raw, 'final_time') && isscalar(raw.final_time) && ...
        isfinite(raw.final_time) && raw.final_time > 0
    timeHorizon = raw.final_time;
end
normalizedStep = 1 / (nodeCount - 1);
targetScale = max(1, abs(problem.xf - problem.x0));
inflatedRadii = problem.obstacle_radii(:).' + problem.safety_margin;

[states, terminalError, weightedError] = rollout(controls);
initialError = terminalError;
acceptedSteps = 0;
damping = 1e-4;

for projectionIteration = 1:problem.terminal_projection_max_iterations
    if terminalError <= problem.terminal_projection_tolerance
        break;
    end
    residual = (states(:, end) - problem.xf) ./ targetScale;
    variableCount = numel(controls);
    sensitivity = zeros(numel(residual), variableCount);
    perturbation = 1e-5;
    for variableIndex = 1:variableCount
        perturbed = controls;
        perturbed(variableIndex) = perturbed(variableIndex) + perturbation;
        perturbedStates = rolloutStates(perturbed);
        sensitivity(:, variableIndex) = ...
            ((perturbedStates(:, end) - states(:, end)) ./ targetScale) / perturbation;
    end

    normalMatrix = sensitivity.' * sensitivity + damping * eye(variableCount);
    direction = -normalMatrix \ (sensitivity.' * residual);
    accepted = false;
    for stepLength = [1, 0.5, 0.25, 0.1, 0.05, 0.01]
        candidateControls = controls + stepLength * reshape(direction, size(controls));
        candidateControls = clampControls(candidateControls, problem.u_max);
        [candidateStates, candidateError, candidateWeightedError] = rollout(candidateControls);
        if candidateWeightedError < weightedError && isSafe(candidateStates)
            controls = candidateControls;
            states = candidateStates;
            terminalError = candidateError;
            weightedError = candidateWeightedError;
            acceptedSteps = acceptedSteps + 1;
            damping = max(1e-8, damping / 3);
            accepted = true;
            break;
        end
    end
    if ~accepted
        damping = damping * 10;
        if damping > 1e8
            break;
        end
    end
end

raw.x = matrixToCells(states);
raw.u = matrixToCells([controls, controls(:, end)]);
raw.final_time = timeHorizon;
projectedObjective = problem.cost.phi(states(:, end), problem.xf);
for nodeIndex = 1:nodeCount-1
    projectedObjective = projectedObjective + problem.cost.L( ...
        states(:, nodeIndex), problem.xf, controls(:, nodeIndex), ...
        timeHorizon, normalizedStep);
end
raw.projected_objective = projectedObjective;
raw.returned_cost = projectedObjective;
if isfield(raw, 'J') && isnumeric(raw.J)
    history = raw.J(:);
    history = history(isfinite(history) & history ~= 0);
    raw.J = [history; projectedObjective];
else
    raw.J = projectedObjective;
end
raw.J_accepted = raw.J;
raw.terminal_projection = struct( ...
    'enabled', true, ...
    'initial_terminal_error', initialError, ...
    'final_terminal_error', terminalError, ...
    'tolerance', problem.terminal_projection_tolerance, ...
    'accepted_steps', acceptedSteps, ...
    'maximum_iterations', problem.terminal_projection_max_iterations, ...
    'minimum_segment_clearance', minimumSegmentClearance(states));
if terminalError <= problem.terminal_projection_tolerance
    raw.primary_termination_reason = stringField(raw, 'termination_reason', ...
        'primary_solver_completed');
    raw.primary_convergence_status = stringField(raw, 'convergence_status', ...
        'primary_status_unreported');
    raw.termination_reason = 'terminal_projection_tolerance_met';
    raw.convergence_status = 'converged_terminal_projection';
else
    raw.termination_reason = 'terminal_projection_stalled';
    raw.convergence_status = 'terminal_projection_tolerance_not_met';
end

    function [rolledStates, errorNorm, scaledErrorNorm] = rollout(candidateControls)
        rolledStates = rolloutStates(candidateControls);
        errorVector = rolledStates(:, end) - problem.xf;
        errorNorm = norm(errorVector);
        scaledErrorNorm = norm(errorVector ./ targetScale);
    end

    function rolledStates = rolloutStates(candidateControls)
        rolledStates = zeros(numel(problem.x0), nodeCount);
        rolledStates(:, 1) = problem.x0;
        for nodeIndex = 1:nodeCount-1
            derivative = problem.dynamics.F(rolledStates(:, nodeIndex), ...
                candidateControls(:, nodeIndex), timeHorizon);
            rolledStates(:, nodeIndex+1) = rolledStates(:, nodeIndex) + ...
                normalizedStep * derivative;
        end
    end

    function safe = isSafe(candidateStates)
        safe = minimumSegmentClearance(candidateStates) >= ...
            -problem.constraint_tolerance;
    end

    function clearance = minimumSegmentClearance(candidateStates)
        positionDimension = size(problem.obstacle_centers, 1);
        positions = candidateStates(1:positionDimension, :);
        clearance = Inf;
        for obstacleIndex = 1:size(problem.obstacle_centers, 2)
            center = problem.obstacle_centers(:, obstacleIndex);
            for segmentIndex = 1:size(positions, 2)-1
                segment = positions(:, segmentIndex+1) - positions(:, segmentIndex);
                offset = center - positions(:, segmentIndex);
                denominator = segment.' * segment;
                if denominator > 0
                    fraction = min(1, max(0, (offset.' * segment) / denominator));
                else
                    fraction = 0;
                end
                closest = positions(:, segmentIndex) + fraction * segment;
                clearance = min(clearance, norm(closest - center) - ...
                    inflatedRadii(obstacleIndex));
            end
        end
    end
end

function matrix = trajectoryMatrix(value)
if iscell(value)
    matrix = cell2mat(reshape(value, 1, []));
else
    matrix = value;
end
if isvector(matrix), matrix = matrix(:).'; end
end

function cells = matrixToCells(matrix)
cells = mat2cell(matrix, size(matrix, 1), ones(1, size(matrix, 2))).';
end

function controls = clampControls(controls, bounds)
bounds = bounds(:);
if isscalar(bounds), bounds = repmat(bounds, size(controls, 1), 1); end
controls = min(bounds, max(-bounds, controls));
end

function value = stringField(input, fieldName, fallback)
value = fallback;
if isfield(input, fieldName) && (ischar(input.(fieldName)) || isstring(input.(fieldName)))
    value = char(input.(fieldName));
end
end
