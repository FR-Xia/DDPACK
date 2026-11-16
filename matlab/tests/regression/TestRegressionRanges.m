classdef TestRegressionRanges < matlab.unittest.TestCase
    methods (Test)
        function linearRegulationRange(testCase)
            rng(4, 'twister');
            problem = ddpack.problems.linearRegulation();
            output = evalc('solution = ddpack.solve(problem, ddpack.Options(''Algorithm'',''free_time_pddp'',''MaxIterations'',80,''Alpha'',0.2));'); %#ok<NASGU>
            testCase.verifyTrue(solution.success);
            testCase.verifyGreaterThan(solution.terminal_error, 1e-4);
            testCase.verifyLessThan(solution.terminal_error, 0.01);
            testCase.verifySize(solution.states, [2, 50]);
        end
    end
end
