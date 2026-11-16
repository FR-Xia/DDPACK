classdef TestPublicAPI < matlab.unittest.TestCase
    methods (Test)
        function versionIsSemantic(testCase)
            testCase.verifyEqual(ddpack.version(), '1.1.0');
            testCase.verifyEqual(DDPACK.version(), '1.1.0');
        end

        function invalidAlgorithmIsRejected(testCase)
            testCase.verifyError(@() ddpack.Options('Algorithm', 'unknown'), ...
                'MATLAB:InputParser:ArgumentFailedValidation');
        end

        function invalidTimeGridIsRejected(testCase)
            problem = ddpack.problems.linearRegulation();
            problem.dt = problem.tf;
            testCase.verifyError(@() ddpack.solve(problem, ddpack.Options()), ...
                'ddpack:solve:InvalidTimeGrid');
        end

        function defaultConstructorIsValid(testCase)
            optimizer = DDPACK();
            testCase.verifyEqual(optimizer.algorithm_name, 'free_time_pddp');
            testCase.verifyEqual(numel(optimizer.x0), numel(optimizer.xf));
        end

        function pointMassDerivativeDimensions(testCase)
            dynamics = PointMassDynamics([0, 1; -1, -0.4], [0; 0.2], [0; 0]);
            state = [0.3; -0.2]; control = 0.4; step = 1e-6;
            analytic = dynamics.A;
            jacobian = zeros(2);
            for column = 1:2
                perturbation = zeros(2, 1); perturbation(column) = step;
                plus = dynamics.F(state + perturbation, control);
                minus = dynamics.F(state - perturbation, control);
                jacobian(:, column) = (plus - minus) / (2 * step);
            end
            testCase.verifySize(dynamics.F(state, control), [2, 1]);
            testCase.verifyEqual(analytic, jacobian, 'AbsTol', 2e-6);
        end

        function saveLoadRoundTrip(testCase)
            raw = struct('x', {{[1; 0], [0; 0]}}, 'u', {{0, 0}}, ...
                't', [0, 1], 'J', [2, 1], 'error', 0, 'iter', 2);
            problem = ddpack.problems.linearRegulation();
            problem.tf = 1; problem.dt = 0.5;
            problem.x0 = [1; 0]; problem.xf = [0; 0];
            problem.dynamics_defect_tolerance = 2;
            problem.obstacle_centers = [];
            problem.obstacle_radii = [];
            problem.safety_margin = 0;
            problem.require_cost_decrease = true;
            solution = ddpack.normalizeSolution(raw, 'free_time_pddp', problem, 0.01);
            filename = [tempname, '.mat'];
            cleanup = onCleanup(@() deleteIfPresent(filename));
            ddpack.save(solution, filename);
            restored = ddpack.load(filename);
            testCase.verifyEqual(restored.states, solution.states);
            testCase.verifyEqual(restored.metadata.schema_version, 1);
            testCase.verifyEqual(restored.algorithm_id, 'free_time_pddp');
        end
    end
end

function deleteIfPresent(filename)
if isfile(filename), delete(filename); end
end
