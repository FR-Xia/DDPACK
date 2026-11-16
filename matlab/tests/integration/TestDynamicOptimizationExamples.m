classdef TestDynamicOptimizationExamples < matlab.unittest.TestCase
    methods (Test)
        function nonlinearThermalRegulation(testCase)
            outputDir = tempname;
            cleanup = onCleanup(@() removeIfPresent(outputDir)); %#ok<NASGU>
            result = example_thermal_regulation( ...
                'OutputDir', outputDir, 'Visible', false, 'QuickMode', false);
            verifyDynamicExample(testCase, result, outputDir);
            testCase.verifyLessThanOrEqual(result.solution.terminal_error, 5e-5);
            testCase.verifyEqual(result.solution.termination_reason, ...
                'trial_cost_change_below_tolerance');
            testCase.verifyEqual(result.solution.convergence_status, ...
                'converged_by_cost_change');
            testCase.verifyFalse(isfield(result.solution.metadata.problem, ...
                'obstacle_centers') && ...
                ~isempty(result.solution.metadata.problem.obstacle_centers));
        end

        function constrainedEnergyStorage(testCase)
            outputDir = tempname;
            cleanup = onCleanup(@() removeIfPresent(outputDir)); %#ok<NASGU>
            result = example_energy_storage_management( ...
                'OutputDir', outputDir, 'Visible', false, 'QuickMode', false);
            verifyDynamicExample(testCase, result, outputDir);
            testCase.verifyLessThanOrEqual( ...
                result.solution.maximum_state_constraint_violation, 1e-8);
            testCase.verifyGreaterThanOrEqual(min(result.solution.states(:)), 0.25);
            testCase.verifyLessThanOrEqual(result.solution.terminal_error, 5e-5);
            testCase.verifyEqual(result.solution.termination_reason, ...
                'trial_cost_change_below_tolerance');
            testCase.verifyEqual(result.solution.convergence_status, ...
                'converged_by_cost_change');
        end
    end
end

function verifyDynamicExample(testCase, result, outputDir)
testCase.verifyTrue(result.solution.success, result.solution.message);
testCase.verifyEqual(result.solution.algorithm_id, 'penalty_free_time_ddp');
required = {'solution.mat', 'metrics.csv', 'metrics.json', ...
    'trajectory.png', 'trajectory.pdf', 'summary.txt'};
for index = 1:numel(required)
    testCase.verifyTrue(isfile(fullfile(outputDir, required{index})), required{index});
end
summary = fileread(fullfile(outputDir, 'summary.txt'));
testCase.verifySubstring(summary, 'Initial condition');
testCase.verifySubstring(summary, 'Terminal target');
testCase.verifySubstring(summary, 'Constraints');
end

function removeIfPresent(folder)
if isfolder(folder), rmdir(folder, 's'); end
end
