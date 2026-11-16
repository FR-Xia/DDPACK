classdef TestAcceptanceSafety < matlab.unittest.TestCase
    % Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods(Test)
        function terminalConstraintIsChecked(testCase)
            [p,raw]=fixture([0,-1],[-10,-10],[0,.1]);
            p.constraints=StateLowerBoundConstraint(1,0,1);
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
            testCase.verifyFalse(s.success);testCase.verifyEqual(s.metadata.metrics.maximum_state_constraint_violation,1);
        end
        function physicalViolationIsGridIndependent(testCase)
            for h=[.001,.1,1]
                [p,raw]=fixture([-.1,0],[.1/h,.1/h],[0,h]);
                p.constraints=StateLowerBoundConstraint(1,0,1);p.constraint_tolerance=.001;
                s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
                testCase.verifyFalse(s.success);
                testCase.verifyEqual(s.metadata.metrics.maximum_state_constraint_violation,.1,'AbsTol',1e-14);
            end
        end
        function invalidConstraintOutputsAreRejected(testCase)
            [p,raw]=fixture([0,0],[0,0],[0,1]);
            for mode={'nan','inf','complex','empty','matrix'}
                p.constraints=AcceptanceConstraintFixture(mode{1});
                testCase.verifyError(@()ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0), ...
                    'ddpack:metrics:InvalidConstraintValue');
            end
        end
        function variableConstraintShapeIsRejected(testCase)
            [p,raw]=fixture([0,1],[1,1],[0,1]);p.constraints=AcceptanceConstraintFixture('changing');
            testCase.verifyError(@()ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0),'ddpack:metrics:ConstraintDimensions');
        end
        function callbackCauseIsPreserved(testCase)
            [p,raw]=fixture([0,0],[0,0],[0,1]);p.constraints=AcceptanceConstraintFixture('throws');
            try
                ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
                testCase.assertFail('Expected constraint exception.');
            catch e
                testCase.verifyEqual(e.identifier,'ddpack:metrics:ConstraintEvaluation');
                testCase.verifyEqual(e.cause{1}.identifier,'ddpack:test:ConstraintCause');
            end
        end
        function validVectorConstraintPasses(testCase)
            [p,raw]=fixture([0,1],[1,1],[0,1]);p.constraints=AcceptanceConstraintFixture('valid');
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);testCase.verifyTrue(s.success);
        end
        function terminalConstraintUsesLastActiveControl(testCase)
            [p,raw]=fixture([0,1],[1,-10],[0,1]);
            p.constraints=AcceptanceConstraintFixture('held_control');
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
            testCase.verifyTrue(s.success);
            testCase.verifyEqual(s.metadata.metrics.maximum_state_constraint_violation,0);
        end
        function lateDiagnosticsFailureCannotPassWithInfiniteTolerance(testCase)
            [p,raw]=fixture([0,.4,.8],[0,0,0],[0,1,2]);p.dynamics=AcceptanceDynamicsFixture('throws');
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);d=s.metadata.defect_diagnostics;
            testCase.verifyFalse(s.success);testCase.verifyFalse(d.complete);
            testCase.verifyEqual(d.evaluated_step_count,1);testCase.verifyEqual(d.expected_step_count,2);
            testCase.verifyEqual(d.failure_step,2);testCase.verifyEqual(d.failure_identifier,'ddpack:test:DynamicsCause');
            testCase.verifyEqual(d.high_accuracy_absolute_max,Inf);testCase.verifyTrue(isnan(d.high_accuracy_defects(2)));
            testCase.verifySubstring(s.message,'ddpack:test:DynamicsCause');
        end
        function invalidDerivativesDoNotYieldPartialSuccess(testCase)
            for mode={'nan','complex','shape'}
                [p,raw]=fixture([0,.4,.8],[0,0,0],[0,1,2]);p.dynamics=AcceptanceDynamicsFixture(mode{1});
                s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
                testCase.verifyFalse(s.success);testCase.verifyFalse(s.metadata.defect_diagnostics.complete);
                testCase.verifyEqual(s.metadata.defect_diagnostics.failure_identifier,'ddpack:diagnostics:InvalidDerivative');
            end
        end
        function successfulDiagnosticsCoverAllIntervals(testCase)
            [p,raw]=fixture([0,.4,.8],[.4,.4,.4],[0,1,2]);
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);d=s.metadata.defect_diagnostics;
            testCase.verifyTrue(s.success);testCase.verifyTrue(d.complete);
            testCase.verifyEqual(d.evaluated_step_count,2);testCase.verifyEmpty(d.failure_identifier);
        end
        function requiredTimeScaleSignatureRemainsSupported(testCase)
            [p,raw]=fixture([0,1],[0,0],[0,1]);
            p.x0=zeros(3,1);p.xf=[1;0;0];p.dynamics=Dynamics2D_cbf_mspddp(1);
            p.cost=QuadraticCost(eye(3),eye(3),1);raw.x={zeros(3,1),[1;0;0]};
            s=ddpack.normalizeSolution(raw,'penalty_free_time_ddp',p,0);
            testCase.verifyTrue(s.success);testCase.verifyTrue(s.metadata.defect_diagnostics.complete);
        end
    end
end
function [p,raw]=fixture(x,u,time)
p=struct('x0',x(1),'xf',x(end),'tf',time(end),'dt',time(end)/2,'u_max',1000, ...
    'dynamics',PointMassDynamics(0,1,0),'cost',QuadraticCost_fft(0,0,1), ...
    'constraints',{{}},'require_cost_decrease',false);
raw=struct('x',x,'u',u,'t',time,'J',1,'error',0,'convergence_status','converged_test');
end
