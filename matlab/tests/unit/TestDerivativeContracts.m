classdef TestDerivativeContracts < matlab.unittest.TestCase
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods(Test)
        function allFormalModelContractsPassOffOrigin(testCase)
            names={'vibration','planar','terminal_defect','obstacle_2d','obstacle_3d','reactor','pendulum','thermal','storage'};
            for k=1:numel(names)
                [p,o]=comparison_case(names{k});x=.6*p.x0+.4*p.xf+.07*sin((1:numel(p.x0)).');
                u=.13*p.u_max(:);
                report=ddpack.checkDerivatives(p,x,u,'Algorithm',o.Algorithm);
                testCase.verifyTrue(report.success,jsonencode(report));
                testCase.verifyGreaterThan(report.checked_count,10);
            end
        end
        function nonunitTimeScalingIsExplicitlyChecked(testCase)
            p=ddpack.problems.linearRegulation();
            r=ddpack.checkDerivatives(p,[.3;-.2],.17,'TimeScale',2.3,'IntegrationStep',.043);
            testCase.verifyTrue(r.success);testCase.verifyTrue(r.time_scaling_checked);
            testCase.verifyTrue(all(ismember({'dynamics.theta','running.L_t','running.L_xt','running.L_ut','running.L_tt'}, {r.checks.name})));
        end
        function mixedControlCostUsesIndependentCrossChecks(testCase)
            p=struct('xf',zeros(2,1),'cost',MixedDerivativeCost(), ...
                'dynamics',PointMassDynamics([0,1;-.3,-.2],[1,.2;.1,1],zeros(2,1)));
            r=ddpack.checkDerivatives(p,[.3;-.4],[.1;-.2],'Algorithm','penalty_free_time_ddp');
            testCase.verifyTrue(r.success,jsonencode(r));testCase.verifyFalse(r.time_scaling_checked);
            testCase.verifyTrue(all(ismember({'running.L_xu','running.L_ux'}, {r.checks.name})));
        end
        function wrongDerivativeIsDetected(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=BrokenDerivativeDynamics('wrong');
            r=ddpack.checkDerivatives(p,[.2;-.3],.1);c=r.checks(strcmp({r.checks.name},'dynamics.Phi'));
            testCase.verifyFalse(r.success);testCase.verifyFalse(c.passed);
            testCase.verifyEqual(c.category,'derivative');testCase.verifyGreaterThan(c.max_tolerance_ratio,1);
        end
        function wrongDimensionsAreNotBroadcast(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=BrokenDerivativeDynamics('shape');
            r=ddpack.checkDerivatives(p,[.2;-.3],.1);c=r.checks(strcmp({r.checks.name},'dynamics.Phi'));
            testCase.verifyFalse(r.success);testCase.verifyEqual(c.category,'callback_contract');
            testCase.verifySubstring(c.message,'CallbackContract');
        end
        function nonfiniteDerivativeIsDetected(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=BrokenDerivativeDynamics('nonfinite');
            r=ddpack.checkDerivatives(p,[.2;-.3],.1);testCase.verifyFalse(r.success);
        end
        function isolatedNonfiniteCenterValueIsDetected(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=BrokenDerivativeDynamics('value');
            r=ddpack.checkDerivatives(p,[.2;-.3],.1);c=r.checks(strcmp({r.checks.name},'dynamics.F'));
            testCase.verifyFalse(r.success);testCase.verifyFalse(c.passed);
            testCase.verifyEqual(c.category,'value_contract');
            testCase.verifyEqual(r.failed_count,1);
        end
        function callbackErrorsAreIdentifiedInReport(testCase)
            p=ddpack.problems.linearRegulation();p.dynamics=BrokenDerivativeDynamics('throw');
            r=ddpack.checkDerivatives(p,[.2;-.3],.1);c=r.checks(strcmp({r.checks.name},'dynamics.Phi'));
            testCase.verifyFalse(r.success);testCase.verifySubstring(c.message,'ddpack:test:DerivativeCallback');
        end
        function constraintsPassInsideAndOutsideFeasibleSet(testCase)
            p=ddpack.problems.linearRegulation();p.constraints=StateLowerBoundConstraint(1,.1,2);
            for value=[-.4,.1,.6]
                r=ddpack.checkDerivatives(p,[value;.3],.1);
                testCase.verifyTrue(r.success,jsonencode(r));
                testCase.verifyTrue(any(strcmp({r.checks.name},'constraint.1.G_x')));
            end
        end
        function checkerDoesNotChangeRandomStream(testCase)
            p=ddpack.problems.linearRegulation();before=rng;
            ddpack.checkDerivatives(p,[.2;-.3],.1);after=rng;
            testCase.verifyEqual(after,before);
        end
        function invalidStencilIsRejected(testCase)
            p=ddpack.problems.linearRegulation();
            testCase.verifyError(@()ddpack.checkDerivatives(p,[.2;-.3],.1,'DifferenceStep',0), ...
                'MATLAB:InputParser:ArgumentFailedValidation');
        end
        function auditRefusesExistingOutput(testCase)
            testCase.verifyError(@()run_derivative_audit(tempdir),'ddpack:derivatives:ExistingOutput');
        end
    end
end
