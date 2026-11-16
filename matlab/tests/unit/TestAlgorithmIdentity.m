classdef TestAlgorithmIdentity < matlab.unittest.TestCase
    methods (Test)
        function registryHasRequiredFieldsAndUniqueIds(testCase)
            registry = ddpack.algorithmRegistry();
            required = {'algorithm_id', 'public_name', 'short_name', ...
                'legacy_names', 'implementation_status', 'theoretical_scope', ...
                'reference_relation', 'stable_or_experimental', ...
                'public_entry_point', 'legacy_entry_points', 'limitations', ...
                'recommended_citation_text'};
            testCase.verifyEqual(sort(fieldnames(registry)), sort(required(:)));
            testCase.verifyEqual(numel(unique({registry.algorithm_id})), numel(registry));
            testCase.verifyEqual(sum(strcmp({registry.stable_or_experimental}, 'stable')), 6);
            testCase.verifyEqual(sum(strcmp({registry.stable_or_experimental}, 'experimental')), 0);
        end

        function everyLegacyNameResolvesToOnePublicId(testCase)
            registry = ddpack.algorithmRegistry();
            for index = 1:numel(registry)
                for aliasIndex = 1:numel(registry(index).legacy_names)
                    [identity, legacy] = ddpack.resolveAlgorithm( ...
                        registry(index).legacy_names{aliasIndex});
                    testCase.verifyTrue(legacy);
                    testCase.verifyEqual(identity.algorithm_id, registry(index).algorithm_id);
                end
            end
        end

        function legacyPublicNameWarnsAndMatchesNumerically(testCase)
            problem = ddpack.problems.linearRegulation();
            rng(81, 'twister');
            canonical = ddpack.solve(problem, ddpack.Options( ...
                'Algorithm', 'free_time_pddp', 'MaxIterations', 5));
            rng(81, 'twister');
            lastwarn('');
            output = evalc("legacy = ddpack.solve(problem, ddpack.Options('Algorithm','pddp','MaxIterations',5));"); %#ok<NASGU>
            [~, warningId] = lastwarn;
            testCase.verifyEqual(warningId, 'ddpack:DeprecatedAlgorithmName');
            testCase.verifyEqual(legacy.states, canonical.states, 'AbsTol', 0);
            testCase.verifyEqual(legacy.controls, canonical.controls, 'AbsTol', 0);
            testCase.verifyEqual(legacy.objective, canonical.objective, 'AbsTol', 0);
            testCase.verifyEqual(legacy.algorithm_id, 'free_time_pddp');
            testCase.verifyEqual(legacy.legacy_algorithm_name, 'pddp');
        end

        function legacyEntrypointsAreThinWrappers(testCase)
            root = repositoryRoot();
            aliases = {'pddp', 'c_pddp', 'mspddp', ...
                'cbf_mspddp_2d', 'cbf_mspddp_3d', 'fft_cddp'};
            targets = {'free_time_pddp', 'waypoint_pddp', 'terminal_defect_pddp', ...
                'geometric_safety_pddp_2d', 'geometric_safety_pddp_3d', ...
                'penalty_free_time_ddp'};
            for index = 1:numel(aliases)
                text = fileread(fullfile(root, 'src', 'algorithms', [aliases{index} '.m']));
                testCase.verifySubstring(text, 'ddpack:DeprecatedAlgorithmName');
                testCase.verifySubstring(text, targets{index});
                testCase.verifyEmpty(regexp(text, ...
                    '(?m)^\s*(for|while)\s+', 'once'));
            end
        end

        function constraintConventionIsPositiveFeasibility(testCase)
            constraint = StateLowerBoundConstraint(1, 0.25, 1);
            testCase.verifyGreaterThan(constraint.G(0.5, 0, 1), 0);
            testCase.verifyLessThan(constraint.G(0.1, 0, 1), 0);
            [g, gx, gu] = ddpack.feasibilityFromLessEqual(2, [1, -1], 3);
            testCase.verifyEqual(g, -2);
            testCase.verifyEqual(gx, [-1, 1]);
            testCase.verifyEqual(gu, -3);
        end
    end
end

function root = repositoryRoot()
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
