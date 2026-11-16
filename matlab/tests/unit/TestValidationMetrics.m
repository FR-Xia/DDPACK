classdef TestValidationMetrics < matlab.unittest.TestCase
    methods (Test)
        function analyticRolloutDetectsCurvedIntersampleCollision(testCase)
            problem=struct('x0',[0;0;0],'xf',[0;0;2*pi], ...
                'dynamics',Dynamics2D_cbf_mspddp(1),'obstacle_centers',[0;1], ...
                'obstacle_radii',0.1,'safety_margin',0);
            audit=ddpack.verifyGeometricRollout(problem,[2,2],[0,pi]);
            testCase.verifyLessThan(audit.clearance_lower_bound,-0.1);
            testCase.verifyLessThan(audit.terminal_error,1e-12);
            % Zero rate exercises the removable singularity in sin(z)/z.
            problem.xf=[pi;0;0];problem.obstacle_centers=[0;2];
            audit=ddpack.verifyGeometricRollout(problem,[0,0],[0,pi]);
            testCase.verifyLessThan(audit.terminal_error,1e-12);
            testCase.verifyGreaterThan(audit.clearance_lower_bound,1.8);
        end
        function normalizedTimeUsesFinalTime(testCase)
            problem = scalarProblem();
            problem.tf = 10;
            raw = struct('x', {{0, 5}}, 'u', {{0, 0}}, ...
                't', [0, 1], 'Theta', [10; 8], 'final_time', 5, ...
                'J', [2; 1], 'error', 0, 'd', 0);
            solution = ddpack.normalizeSolution(raw, 'terminal_defect_pddp', problem, 0.01);
            testCase.verifyEqual(solution.time, [0, 5], 'AbsTol', 1e-14);
            testCase.verifyLessThan(solution.absolute_dynamics_defect, 1e-12);
        end

        function finalTimeOverridesLaggingHistory(testCase)
            problem = scalarProblem();
            raw = struct('x', {{0, 4}}, 'u', {{0, 0}}, ...
                't', [0, 1], 'Theta', [7; 6], 'final_time', 4, ...
                'J', 1, 'error', 0, 'd', 0);
            solution = ddpack.normalizeSolution(raw, 'geometric_safety_pddp_2d', problem, 0);
            testCase.verifyEqual(solution.metadata.time_grid.optimized_horizon, 4);
        end

        function extraControlsAreTrackedAndAligned(testCase)
            problem = scalarProblem();
            raw = struct('x', {{0, 1, 2}}, 'u', {{0, 0, 0, 9}}, ...
                't', [0, 1, 2], 'J', [2; 1], 'error', 0);
            solution = ddpack.normalizeSolution(raw, 'free_time_pddp', problem, 0);
            testCase.verifySize(solution.controls, [1, 3]);
            testCase.verifyEqual(solution.metadata.native_control_count, 4);
        end

        function nonApplicableMetricsAreNaN(testCase)
            problem = scalarProblem();
            raw = struct('x', {{0, 1}}, 'u', {{0, 0}}, ...
                't', [0, 1], 'J', [2; 1], 'error', 0);
            solution = ddpack.normalizeSolution(raw, 'free_time_pddp', problem, 0);
            testCase.verifyTrue(isnan(solution.min_obstacle_clearance));
            testCase.verifyTrue(isnan(solution.minimum_pairwise_distance));
            testCase.verifyEqual(solution.metadata.metric_applicability.minimum_obstacle_clearance, ...
                'not_applicable');
            testCase.verifySubstring(jsonencode(solution.metadata.metrics), ...
                '"minimum_obstacle_clearance":null');
        end

        function trueThreeDimensionalClearance(testCase)
            problem = scalarProblem();
            problem.x0 = [0; 0; 0]; problem.xf = [2; 0; 0];
            problem.dynamics = PointMassDynamics(zeros(3), zeros(3, 1), [1; 0; 0]);
            problem.obstacle_centers = [1; 0; 1];
            problem.obstacle_radii = 0.25; problem.safety_margin = 0.1;
            states = [0, 1, 2; 0, 0, 0; 0, 0, 0]; controls = zeros(1, 3);
            [metrics, ~] = ddpack.computeMetrics(states, controls, [0, 1, 2], ...
                problem, [], 'free_time_pddp', struct());
            testCase.verifyEqual(metrics.minimum_obstacle_clearance, 0.65, 'AbsTol', 1e-12);
        end

        function segmentClearanceCatchesBetweenNodeCollision(testCase)
            problem = scalarProblem();
            problem.x0 = [-1; 0]; problem.xf = [1; 0];
            problem.dynamics = PointMassDynamics(zeros(2), zeros(2, 1), [1; 0]);
            problem.obstacle_centers = [0; 0];
            problem.obstacle_radii = 0.5;
            states = [-1, 1; 0, 0]; controls = zeros(1, 2);
            [metrics, ~] = ddpack.computeMetrics(states, controls, [0, 1], ...
                problem, [], 'free_time_pddp', struct());
            testCase.verifyEqual(metrics.minimum_node_obstacle_clearance, 0.5, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.minimum_segment_obstacle_clearance, -0.5, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.clearance_minimum_segment_index, 1);
            testCase.verifyEqual(metrics.clearance_segment_fraction, 0.5, ...
                'AbsTol', 1e-12);
        end

        function circleConstraintMatchesPublicContract(testCase)
            constraint = CircleConstraints([0, 0], 1, 0.2);
            outside = constraint.G([2; 0], 0, 0.1);
            inside = constraint.G([0.5; 0], 0, 0.1);
            testCase.verifyGreaterThan(outside, 0);
            testCase.verifyLessThan(inside, 0);
            testCase.verifySize(constraint.G_x([2; 0; 0], 0, 0.1), [1, 3]);
        end

        function defectMaximumLocationIsReported(testCase)
            problem = scalarProblem();
            states = [0, 1, 2.2]; controls = zeros(1, 3);
            diagnostics = ddpack.computeDefectDiagnostics( ...
                states, controls, [0, 1, 2], problem, 'free_time_pddp', struct());
            testCase.verifyEqual(diagnostics.solver_consistent_max_step, 2);
            testCase.verifyEqual(diagnostics.solver_consistent_max_component, 1);
            testCase.verifyEqual(diagnostics.solver_consistent_absolute_max, 0.2, 'AbsTol', 1e-12);
        end

        function strictAcceptedCostsRejectNonImprovingTrials(testCase)
            trials = [10, 8, 8.5, NaN, 7, 7, 9];
            [costs, indices] = ddpack.strictAcceptedCosts(trials);
            testCase.verifyEqual(costs, [10; 8; 7]);
            testCase.verifyEqual(indices, [1; 2; 5]);
        end

        function kinematicFlightDerivativeDimensions(testCase)
            dynamics = KinematicFlightDynamics();
            state = [0; 0; 0; 1.2; 0.1; 0.2]; control = [0.3; -0.2];
            testCase.verifySize(dynamics.F(state, control), [6, 1]);
            testCase.verifySize(dynamics.Phi(state, control, 2, 0.1), [6, 6]);
            testCase.verifySize(dynamics.beta(state, control, 2, 0.1), [6, 2]);
        end

        function kinematicFlightJacobianFiniteDifference(testCase)
            dynamics = KinematicFlightDynamics();
            state = [0; 0; 0; 1.2; 0.1; 0.2]; control = [0.3; -0.2];
            step = 1e-6; numerical = zeros(6);
            for column = 1:6
                perturbation = zeros(6, 1); perturbation(column) = step;
                numerical(:, column) = (dynamics.F(state + perturbation, control) - ...
                    dynamics.F(state - perturbation, control)) / (2*step);
            end
            analytic = (dynamics.Phi(state, control, 1, 1) - eye(6));
            testCase.verifyEqual(analytic, numerical, 'AbsTol', 2e-6);
        end

        function periodicTerminalErrorUsesWrappedAngle(testCase)
            problem = scalarProblem();
            problem.x0 = [0; 0];
            problem.xf = [0; 0];
            problem.dynamics = InvertedPendulumDynamics(1, 1, 9.81, 0.1);
            problem.cost = InvertedPendulumCost(eye(2), eye(2), 1, [0; 0]);
            problem.terminal_error_function = @(state, target) norm([ ...
                mod(state(1) - target(1) + pi, 2*pi) - pi; ...
                state(2) - target(2)]);
            states = [0, 2*pi - 1e-3; 0, 0];
            controls = zeros(1, 2);
            [metrics, ~] = ddpack.computeMetrics(states, controls, [0, 1], ...
                problem, [2; 1], 'waypoint_pddp', struct());
            testCase.verifyEqual(metrics.terminal_error, 1e-3, 'AbsTol', 1e-12);
        end

        function invisiblePlotIsCreated(testCase)
            problem = scalarProblem();
            raw = struct('x', {{0, 1}}, 'u', {{0, 0}}, ...
                't', [0, 1], 'J', [2; 1], 'error', 0);
            solution = ddpack.normalizeSolution(raw, 'free_time_pddp', problem, 0);
            figureHandle = ddpack.plot(solution, 'Visible', false);
            cleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
            testCase.verifyEqual(char(figureHandle.Visible), 'off');
        end

        function invalidThreeDimensionalObstacleIsRejected(testCase)
            problem = scalarProblem();
            problem.x0 = zeros(6, 1); problem.xf = zeros(6, 1);
            problem.dynamics = KinematicFlightDynamics();
            problem.cost = QuadraticCost(eye(6), eye(6), eye(2));
            problem.u_max = [1; 1];
            problem.obstacle_centers = [1; 2]; problem.obstacle_radii = 1;
            testCase.verifyError(@() ddpack.solve(problem, ...
                ddpack.Options('Algorithm', 'geometric_safety_pddp_3d')), ...
                'ddpack:solve:ObstacleDimension');
        end
    end
end

function problem = scalarProblem()
problem = struct('name', 'scalar_test', 'x0', 0, 'xf', 1, ...
    'tf', 2, 'dt', 1, ...
    'dynamics', PointMassDynamics(0, 0, 1), ...
    'cost', QuadraticCost(1, 1, 1), 'u_max', 1, ...
    'constraints', {{}}, 'obstacle_centers', [], 'obstacle_radii', [], ...
    'safety_margin', 0, 'geometric_enforcement_margin', 0, ...
    'safety_gain', 0.5, ...
    'terminal_tolerance', 10, 'constraint_tolerance', 1e-8, ...
    'initial_tolerance', 1e-10, 'dynamics_defect_tolerance', 1, ...
    'independent_dynamics_defect_tolerance', Inf, ...
    'require_cost_decrease', false);
end
