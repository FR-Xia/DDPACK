classdef TestDefectLineSearch < matlab.unittest.TestCase
    methods(Test)
        function overshootBacktracksUsingActualMerit(testCase)
            [x,u,dyn,cost,direction,geometry]=fixture(1,8);
            [nextX,nextU,tf,r]=ddpack.defectLineSearch(x,u,1,1,dyn,cost,10,direction,1,.5,2,10,.95,geometry);
            testCase.verifyTrue(r.accepted);
            testCase.verifyEqual(r.trials(end,2),.125,'AbsTol',1e-15);
            testCase.verifyEqual(r.trials(1:3,end),zeros(3,1));
            testCase.verifyEqual(nextX{end},1,'AbsTol',1e-14);
            testCase.verifyEqual(nextU{1},1,'AbsTol',1e-14);
            testCase.verifyEqual(tf,1);
            testCase.verifyEqual(r.cost_after,.05,'AbsTol',1e-14);
            testCase.verifyEqual(r.merit_after,r.cost_after,'AbsTol',1e-14);
            testCase.verifyLessThan(r.merit_after,r.merit_before);
        end
        function rejectedTrialsLeaveNominalUntouched(testCase)
            [x,u,dyn,cost,direction,geometry]=fixture(0,1);
            direction.time_change=1;
            [nextX,nextU,tf,r]=ddpack.defectLineSearch(x,u,1,0,dyn,cost,10,direction,1,.5,1.1,10,.95,geometry);
            testCase.verifyFalse(r.accepted);
            testCase.verifyEqual(nextX,x);testCase.verifyEqual(nextU,u);testCase.verifyEqual(tf,1);
            testCase.verifyEqual(size(r.trials,1),24);
            testCase.verifyEqual(r.trials(:,end),zeros(24,1));
            testCase.verifyLessThanOrEqual(r.trials(:,5),1.1*ones(24,1));
            testCase.verifyEqual(r.cost_after,0);
        end
        function nonfiniteCandidatesAreRejected(testCase)
            [x,u,~,cost,direction,geometry]=fixture(0,1);
            dyn=struct('F',@(state,control,time) time*(control+realmax*(control>0)*realmax));
            [nextX,nextU,tf,r]=ddpack.defectLineSearch(x,u,1,0,dyn,cost,10,direction,1,.5,2,10,.95,geometry);
            testCase.verifyFalse(r.accepted);
            testCase.verifyEqual(nextX,x);testCase.verifyEqual(nextU,u);testCase.verifyEqual(tf,1);
            testCase.verifyEqual(r.trials(:,end-1:end),zeros(24,2));
        end
        function oneIterationReturnsEvaluatedAcceptedState(testCase)
            for name={'terminal_defect','obstacle_2d','obstacle_3d'}
                [problem,base]=comparison_case(name{1});
                problem.terminal_projection_max_iterations=0;problem.geometric_refinement_iterations=0;
                options=ddpack.Options('Algorithm',base.Algorithm,'MaxIterations',1, ...
                    'Epsilon',base.Epsilon,'Rho',base.Rho,'UpdateEpsilon',base.UpdateEpsilon);
                text=evalc('s=ddpack.solve(problem,options);'); %#ok<NASGU>
                raw=s.raw;trials=raw.line_search.trials;
                testCase.verifyNotEmpty(trials);
                testCase.verifyEqual(trials(:,12),trials(:,8)+trials(:,7).*trials(:,10)+trials(:,16).*trials(:,14),'RelTol',1e-12);
                testCase.verifyEqual(trials(:,13),trials(:,9)+trials(:,7).*trials(:,11)+trials(:,16).*trials(:,15),'RelTol',1e-12);
                accepted=trials(:,end)==1;
                testCase.verifyLessThan(trials(accepted,13),trials(accepted,12));
                expected=problem.cost.phi(s.states(:,end),problem.xf);
                for k=1:size(s.states,2)-1
                    expected=expected+problem.cost.L(s.states(:,k),problem.xf,s.controls(:,k),s.time(end),1/(size(s.states,2)-1));
                end
                testCase.verifyEqual(s.objective,expected,'AbsTol',1e-9);
                testCase.verifyEqual(raw.returned_cost,expected,'AbsTol',1e-9);
                testCase.verifyEqual(numel(raw.J),numel(raw.M));
                testCase.verifyEqual(numel(raw.J),numel(raw.Theta));
                testCase.verifyLessThan(s.absolute_dynamics_defect,1e-12);
            end
        end
    end
end

function [x,u,dyn,cost,direction,geometry]=fixture(target,feed)
x={0;0;0};u={0;0;0};dyn=PointMassDynamics(0,1,0);cost=QuadraticCost(1,0,.1);
direction=struct('feedforward',{{feed;feed;0}},'feedback',{{0;0;0}},'time_gain',{{0;0;0}},'time_change',0);
geometry=struct('dimension',0,'centers',[],'radii',[],'gain',0);
assert(isscalar(target));
end
