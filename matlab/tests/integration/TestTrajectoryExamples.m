classdef TestTrajectoryExamples < matlab.unittest.TestCase
    methods (Test)
        function planarPointMass(testCase)
            verifyExample(testCase, @example_planar_point_mass_trajectory, false);
        end

        function terminalDefectTrajectory(testCase)
            result = verifyExample(testCase, ...
                @example_terminal_defect_trajectory, false);
            testCase.verifyLessThan(result.boundary_rollout_defect, 1e-10);
        end

        function geometricSafetyTwoDimensional(testCase)
            result = verifyExample(testCase, ...
                @example_geometric_safety_pddp_2d_obstacle_avoidance, true);
            testCase.verifyLessThan(min(result.straight_reference_clearances), 0);
            verifyProjectedTerminal(testCase, result, 0.005);
        end

        function geometricSafetyThreeDimensional(testCase)
            result = verifyExample(testCase, ...
                @example_geometric_safety_pddp_3d_obstacle_avoidance, true);
            testCase.verifyLessThan(result.straight_reference_clearance, 0);
            verifyProjectedTerminal(testCase, result, 0.005);
        end

        function penaltyFreeTimeReactor(testCase)
            result = verifyExample(testCase, ...
                @example_penalty_free_time_reactor_regulation, false);
            testCase.verifyGreaterThan(numel(result.solution.trial_cost_history), ...
                numel(result.solution.accepted_cost_history));
            testCase.verifyLessThan(result.solution.accepted_cost_history(end), ...
                result.solution.accepted_cost_history(1));
            testCase.verifyEqual(result.solution.raw.returned_cost, ...
                result.solution.raw.best_accepted_cost, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.solution.raw.accepted_count + ...
                result.solution.raw.rejected_count, ...
                numel(result.solution.trial_cost_history));
            testCase.verifyEqual(result.solution.accepted_objective, ...
                result.solution.raw.best_accepted_cost, 'AbsTol', 1e-12);
            testCase.verifyEqual( ...
                result.solution.metadata.metrics.acceptance_semantics, ...
                'explicit_strict_best_accepted_history');
        end
    end
end

function verifyProjectedTerminal(testCase, result, tolerance)
testCase.verifyLessThanOrEqual(result.solution.terminal_error, tolerance);
testCase.verifyEqual(result.solution.termination_reason, ...
    'geometric_refinement_feasible');
testCase.verifyEqual(result.solution.convergence_status, ...
    'converged_geometric_refinement');
refinement=result.solution.raw.geometric_refinement;
testCase.verifyGreaterThan(refinement.selected_start,0);
testCase.verifyLessThan(refinement.returned_objective,refinement.baseline_objective);
testCase.verifyLessThan(refinement.returned_rate_cost,refinement.baseline_rate_cost);
testCase.verifyGreaterThanOrEqual(result.solution.time(end),5.0);
testCase.verifyLessThanOrEqual(result.solution.time(end),6.0);
testCase.verifyLessThan(diff(refinement.penalty_history),zeros(1,numel(refinement.penalty_history)-1));
testCase.verifyTrue(result.solution.raw.terminal_projection.enabled);
projection=result.solution.raw.terminal_projection;
if projection.accepted_steps>0
    testCase.verifyLessThan(projection.final_terminal_error,projection.initial_terminal_error);
    testCase.verifyGreaterThanOrEqual(projection.minimum_segment_clearance,-result.solution.metadata.problem.constraint_tolerance);
else
    % An inadmissible projection must preserve its nominal endpoint; the
    % independent refinement stage is checked against the physical bounds above.
    testCase.verifyEqual(projection.final_terminal_error,projection.initial_terminal_error,'AbsTol',1e-12);
end
if numel(result.solution.accepted_cost_history)>1
    testCase.verifyLessThanOrEqual(diff(result.solution.accepted_cost_history), ...
        zeros(numel(result.solution.accepted_cost_history)-1, 1) + 1e-12);
end
end

function result = verifyExample(testCase, exampleFunction, obstacleExpected)
outputDir = tempname;
cleanup = onCleanup(@() removeIfPresent(outputDir)); %#ok<NASGU>
result = exampleFunction('OutputDir', outputDir, 'Visible', false, 'QuickMode', false);
testCase.verifyTrue(result.solution.success, result.solution.message);
testCase.verifyTrue(result.solution.finite_values_pass);
testCase.verifyTrue(result.solution.dimension_check_pass);
testCase.verifyLessThanOrEqual(result.solution.absolute_dynamics_defect, 1e-10);
if obstacleExpected
    testCase.verifyGreaterThanOrEqual(result.solution.min_obstacle_clearance, 0);
else
    testCase.verifyTrue(isnan(result.solution.min_obstacle_clearance));
end
testCase.verifyTrue(isfile(fullfile(outputDir, 'trajectory.png')));
testCase.verifyTrue(isfile(fullfile(outputDir, 'trajectory.pdf')));
testCase.verifyTrue(isfile(fullfile(outputDir, 'defect_diagnostics.csv')));
testCase.verifyTrue(isfile(fullfile(outputDir, 'metrics.json')));
if ~strcmp(result.solution.algorithm, 'penalty_free_time_ddp')
    testCase.verifyEqual( ...
        result.solution.metadata.metrics.best_accepted_objective, ...
        result.solution.metadata.metrics.returned_objective, 'AbsTol', 1e-12);
end
end

function removeIfPresent(folder)
if isfolder(folder), rmdir(folder, 's'); end
end
