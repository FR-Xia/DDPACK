classdef TestReturnRefinementTime < matlab.unittest.TestCase
    % Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods(Test)
        function waypointBudgetObjectiveMatchesReturnedTrajectory(testCase)
            checkBudget(testCase,'planar');
        end
        function freeTimeBudgetObjectiveMatchesReturnedTrajectory(testCase)
            checkBudget(testCase,'vibration');
        end
        function freeWaypointBudgetObjectiveMatchesReturnedTrajectory(testCase)
            checkBudget(testCase,'planar',true);
        end
        function lastBudgetUpdateOverflowReturnsFiniteNominal(testCase)
            p=ddpack.problems.linearRegulation();p.x0=zeros(2,1);p.xf=ones(2,1);
            p.cost=OverflowCost();p.require_cost_decrease=false;
            s=ddpack.solve(p,ddpack.Options('Algorithm','waypoint_pddp','MaxIterations',1));
            testCase.verifyEqual(s.status,'numerical_failure');
            testCase.verifyEqual(s.controls,zeros(size(s.controls)));
            testCase.verifyTrue(isfinite(s.objective));
            testCase.verifyEqual(s.objective,s.raw.J(end),'AbsTol',1e-9);
        end
        function objectiveUsesReturnedCostRatherThanLastHistory(testCase)
            [p,r]=scalarResult(1);r.J=[10;5];r.initial_cost=10;r.returned_cost=12;
            p.require_cost_decrease=true;
            s=ddpack.normalizeSolution(r,'penalty_free_time_ddp',p,0);
            testCase.verifyEqual(s.objective,12);testCase.verifyEqual(s.trial_cost_history,[10;5]);
            testCase.verifyFalse(s.success);testCase.verifyFalse(s.metadata.metrics.cost_decreased);
        end
        function infeasibleCheapIncumbentCanBeReplaced(testCase)
            [p,r]=refinementProblem();p.require_cost_decrease=false;
            s=ddpack.refineGeometricTrajectory(r,p);d=s.geometric_refinement;
            testCase.verifyFalse(d.baseline_feasible);testCase.verifyGreaterThan(d.selected_start,0);
            testCase.verifyGreaterThan(d.returned_objective,d.baseline_objective);
            checked=ddpack.normalizeSolution(s,'geometric_safety_pddp_2d',p,0);
            testCase.verifyTrue(checked.metadata.metrics.numerical_checks_pass);
        end
        function requiredDecreaseUsesOriginalInitialCost(testCase)
            [p,r]=refinementProblem();p.require_cost_decrease=true;r.initial_cost=100;
            s=ddpack.refineGeometricTrajectory(r,p);d=s.geometric_refinement;
            testCase.verifyGreaterThan(d.selected_start,0);
            testCase.verifyGreaterThan(d.returned_objective,d.baseline_objective);
            testCase.verifyLessThan(d.returned_objective,r.initial_cost);
        end
        function requiredDecreaseCanRejectFeasibleCandidates(testCase)
            [p,r]=refinementProblem();p.require_cost_decrease=true;
            s=ddpack.refineGeometricTrajectory(r,p);
            testCase.verifyEqual(s.geometric_refinement.selected_start,0);
            testCase.verifyTrue(any(s.geometric_refinement.starts(:,7)>0));
        end
        function feasibleCheapIncumbentIsRetained(testCase)
            [p,r]=refinementProblem();p.require_cost_decrease=false;p.obstacle_centers=[100;100];
            s=ddpack.refineGeometricTrajectory(r,p);
            testCase.verifyTrue(s.geometric_refinement.baseline_feasible);
            testCase.verifyEqual(s.geometric_refinement.selected_start,0);
        end
        function refinementStillEnforcesStateConstraints(testCase)
            [p,r]=refinementProblem();p.require_cost_decrease=false;
            p.constraints=StateLowerBoundConstraint(2,.5,3);
            s=ddpack.refineGeometricTrajectory(r,p);
            testCase.verifyEqual(s.geometric_refinement.selected_start,0);
            testCase.verifyFalse(any(s.geometric_refinement.starts(:,7)>0));
        end
        function timeBoundsAreCheckedAtAcceptance(testCase)
            for duration=[2,7]
                [p,r]=scalarResult(duration);p.minimum_time=5;p.maximum_time=6;
                s=ddpack.normalizeSolution(r,'penalty_free_time_ddp',p,0);
                testCase.verifyFalse(s.success);testCase.verifyFalse(s.metadata.metrics.time_bounds_pass);
                testCase.verifyGreaterThan(s.metadata.metrics.time_bound_violation,0);
                testCase.verifySubstring(s.message,'time-bound violation');
            end
        end
        function exactTimeBoundariesPass(testCase)
            for duration=[5,6]
                [p,r]=scalarResult(duration);p.minimum_time=5;p.maximum_time=6;
                s=ddpack.normalizeSolution(r,'penalty_free_time_ddp',p,0);
                testCase.verifyTrue(s.success);testCase.verifyTrue(s.metadata.metrics.time_bounds_pass);
            end
        end
        function unsupportedTimeBoundsAreRejected(testCase)
            p=ddpack.problems.linearRegulation();p.minimum_time=5;p.maximum_time=6;
            for algorithm={'free_time_pddp','waypoint_pddp','terminal_defect_pddp','penalty_free_time_ddp'}
                testCase.verifyError(@()ddpack.solve(p,ddpack.Options('Algorithm',algorithm{1})), ...
                    'ddpack:solve:UnsupportedTimeBounds');
            end
        end
        function effectiveGeometricBoundsMustBeOrdered(testCase)
            [p,~]=refinementProblem();p.minimum_time=[];p.maximum_time=4;
            p.geometric_refinement_iterations=0;
            testCase.verifyError(@()ddpack.solve(p,ddpack.Options('Algorithm','geometric_safety_pddp_2d')), ...
                'ddpack:solve:TimeBounds');
        end
    end
end
function checkBudget(testCase,name,freeWaypoint)
if nargin<3,freeWaypoint=false;end
[p,original]=comparison_case(name);
o=ddpack.Options('Algorithm',original.Algorithm,'MaxIterations',1,'Alpha',original.Alpha, ...
    'Epsilon',original.Epsilon,'Rho',original.Rho,'UpdateEpsilon',original.UpdateEpsilon);
if freeWaypoint,p.freetime=1;end
s=ddpack.solve(p,o);
d=s.states-p.xf;h=diff(s.time);
value=.5*d(:,end).'*p.cost.Q_f*d(:,end)+ ...
    .5*sum(h.*(sum(d(:,1:end-1).*(p.cost.Q_s*d(:,1:end-1)),1)+ ...
    sum(s.controls(:,1:end-1).*(p.cost.R*s.controls(:,1:end-1)),1)));
testCase.verifyEqual(s.objective,value,'AbsTol',1e-9);
testCase.verifyEqual(s.raw.returned_cost,value,'AbsTol',1e-9);
testCase.verifyGreaterThan(abs(s.objective-s.trial_cost_history(end)),1e-8);
testCase.verifyFalse(s.success);
end
function [p,r]=scalarResult(duration)
p=struct('x0',0,'xf',1,'tf',duration,'dt',duration/2,'u_max',10, ...
    'dynamics',PointMassDynamics(0,1,0),'cost',QuadraticCost_fft(0,0,1), ...
    'require_cost_decrease',false);
r=struct('x',[0,1],'u',[1/duration,1/duration],'t',[0,duration], ...
    'J',1,'error',0,'convergence_status','converged_test');
end
function [p,r]=refinementProblem()
[p,~]=comparison_case('obstacle_2d');n=floor(p.tf/p.dt);
p.cost=QuadraticCost(eye(3),zeros(3),1);p.geometric_refinement_iterations=80;
r=struct('u',{repmat({0},n,1)},'final_time',5,'J',0,'initial_cost',0,'error',0);
end
