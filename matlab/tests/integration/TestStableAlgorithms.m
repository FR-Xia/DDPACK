classdef TestStableAlgorithms < matlab.unittest.TestCase
    methods (Test)
        function freeTimePddpEndToEnd(testCase)
            outputDir = tempname;
            cleanup = onCleanup(@() removeIfPresent(outputDir));
            result = example_free_time_pddp_vibration_regulation( ...
                'OutputDir', outputDir, 'Visible', false, 'QuickMode', false);
            testCase.verifyTrue(result.solution.success);
            testCase.verifyLessThan(result.solution.terminal_error, 0.01);
            testCase.verifyLessThanOrEqual(result.solution.dynamics_defect, 1e-6);
            testCase.verifyTrue(isfile(fullfile(outputDir, 'trajectory.png')));
        end

        function waypointPddpEndToEnd(testCase)
            outputDir = tempname;
            cleanup = onCleanup(@() removeIfPresent(outputDir));
            result = example_waypoint_pddp_pendulum( ...
                'OutputDir', outputDir, 'Visible', false, 'QuickMode', false);
            testCase.verifyTrue(result.solution.success);
            testCase.verifyLessThan(result.solution.terminal_error, 0.01);
            testCase.verifyLessThanOrEqual(result.solution.max_constraint_violation, 1e-8);
            testCase.verifyLessThanOrEqual(result.solution.dynamics_defect, 1e-6);
        end
    end
end

function removeIfPresent(folder)
if isfolder(folder), rmdir(folder, 's'); end
end
