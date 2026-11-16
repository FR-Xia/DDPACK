classdef TestNumericalRobustness < matlab.unittest.TestCase
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods(Test)
        function conditionedSystemHasSmallResidual(testCase)
            h=[4,1;1,3];expected=[1,2;-2,3];rhs=h*expected;
            [answer,used,info]=ddpack.solveControlSystem(h,rhs);
            testCase.verifyEqual(answer,expected,'AbsTol',1e-13);
            testCase.verifyEqual(used,h);testCase.verifyEqual(info.regularized_solves,0);
        end
        function singularAndIndefiniteSystemsAreDamped(testCase)
            for h={zeros(2),diag([-2,1]),diag([1e-16,1])}
                [answer,used,info]=ddpack.solveControlSystem(h{1},eye(2));
                testCase.verifyTrue(all(isfinite(answer(:))));
                testCase.verifyGreaterThan(info.maximum_damping,0);
                testCase.verifyEqual(info.regularized_solves,1);
                testCase.verifyLessThan(norm(used*answer-eye(2),Inf),1e-10);
            end
        end
        function nonfiniteAndInconsistentSystemsAreRejected(testCase)
            testCase.verifyError(@()ddpack.solveControlSystem(NaN,1),'ddpack:numerics:NonfiniteLinearSystem');
            testCase.verifyError(@()ddpack.solveControlSystem(1,Inf),'ddpack:numerics:NonfiniteLinearSystem');
            testCase.verifyError(@()ddpack.solveControlSystem(eye(2),ones(3,1)),'ddpack:numerics:LinearSystemDimensions');
        end
        function singularTimeReturnsFiniteFailedNominal(testCase)
            p=struct('x0',0,'xf',0,'tf',2,'dt',.2,'interval',10, ...
                'dynamics',PointMassDynamics(0,0,0),'cost',QuadraticCost_fft(0,0,1), ...
                'u_max',1,'require_cost_decrease',false,'terminal_tolerance',1);
            s=ddpack.solve(p,ddpack.Options('Algorithm','penalty_free_time_ddp','MaxIterations',2));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.termination_reason,'numerical_failure');
            testCase.verifyEqual(s.raw.numerical_diagnostics.failure_identifier,'ddpack:numerics:SingularTimeHessian');
            testCase.verifyEqual(s.states,zeros(1,11));testCase.verifyEqual(s.controls,zeros(1,11));
            testCase.verifyEqual(s.time(end),2);testCase.verifyEqual(s.raw.error,1);
            testCase.verifyTrue(s.metadata.metrics.numerical_checks_pass);
            testCase.verifyFalse(s.metadata.metrics.solver_convergence_pass);
        end
        function timeNewtonSolveAllowsNegativeButRejectsZero(testCase)
            testCase.verifyEqual(ddpack.checkedTimeSolve(-2,[2,4]),[-1,-2]);
            testCase.verifyError(@()ddpack.checkedTimeSolve(0,1),'ddpack:numerics:SingularTimeHessian');
            testCase.verifyError(@()ddpack.checkedTimeSolve(Inf,1),'ddpack:numerics:SingularTimeHessian');
        end
        function budgetExhaustionDoesNotClaimConvergence(testCase)
            p=ddpack.problems.linearRegulation();p.require_cost_decrease=false;p.terminal_tolerance=1e6;
            s=ddpack.solve(p,ddpack.Options('MaxIterations',1));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'feasible_not_converged');
            testCase.verifyEqual(s.termination_reason,'maximum_iterations_reached');
            testCase.verifyFalse(s.metadata.metrics.solver_convergence_pass);
        end
        function programmingErrorsAreRethrown(testCase)
            try
                error('ddpack:test:CallbackBug','Model implementation error');
            catch e
            end
            testCase.verifyError(@()ddpack.numericalFailureSolution(struct(),e,struct()),'ddpack:test:CallbackBug');
        end
        function backwardRetriesStopAtLimit(testCase)
            p=ddpack.problems.linearRegulation();p.cost=NeverPositiveCost();
            s=ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',2));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.raw.numerical_diagnostics.failure_identifier,'ddpack:numerics:BackwardRetriesExhausted');
            testCase.verifyEqual(s.raw.numerical_diagnostics.backward_retries,79);
        end
        function nonfiniteCandidateRollsBackControlsAndStates(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=OverflowDynamics();
            p.x0=zeros(2,1);p.xf=ones(2,1);p.require_cost_decrease=false;
            s=ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',2));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.states,zeros(size(s.states)));
            testCase.verifyEqual(s.controls,zeros(size(s.controls)));
        end
        function overflowingObjectiveReturnsLastFiniteNominal(testCase)
            p=ddpack.problems.linearRegulation();p.cost=OverflowCost();
            p.x0=zeros(2,1);p.xf=ones(2,1);p.require_cost_decrease=false;
            s=ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',3));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.controls,zeros(size(s.controls)));
            testCase.verifyTrue(isfinite(s.metadata.metrics.returned_objective));
        end
        function laterRunningCostOverflowPreservesCompleteNominal(testCase)
            p=delayedOverflowProblem(true);
            s=ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',3));
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.controls,zeros(size(s.controls)));
            testCase.verifyEqual(s.states(2,:),zeros(1,size(s.states,2)));
            objective=.5*sum((s.states(:,end)-p.xf).^2);
            h=1/(size(s.states,2)-1);
            for k=1:size(s.states,2)-1
                objective=objective+p.cost.L(s.states(:,k),p.xf,s.controls(:,k),s.time(end),h);
            end
            testCase.verifyTrue(isfinite(objective));
            testCase.verifyEqual(s.metadata.metrics.returned_objective,objective,'AbsTol',1e-10);
        end
        function initialLaterRunningCostOverflowCannotCreateNominal(testCase)
            p=delayedOverflowProblem(false);
            testCase.verifyError(@()ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',2)), ...
                'ddpack:numerics:NonfiniteTrajectory');
        end
    end
end

function p=delayedOverflowProblem(requireControl)
p=struct('x0',zeros(2,1),'xf',[10;1],'tf',10,'dt',1, ...
    'dynamics',PointMassDynamics(zeros(2),[0;1],[1;0]), ...
    'cost',DelayedOverflowCost(requireControl),'u_max',1,'require_cost_decrease',false);
end
