classdef TestDocumentation < matlab.unittest.TestCase
    methods (Test)
        function requiredRepositoryFilesExist(testCase)
            root = repositoryRoot();
            required = {'README.md', 'README.rst', 'LICENSE.txt', 'CITATION.cff', ...
                fullfile('DDPACK_Manual', 'DDPACK_Manual.tex'), ...
                fullfile('DDPACK_Manual', 'DDPACK_Manual.pdf'), ...
                fullfile('docs', 'VALIDATION.rst'), ...
                'codemeta.json', ...
                fullfile('.github', 'workflows', 'ci.yml')};
            testCase.verifyTrue(all(cellfun(@(path) ...
                isfile(fullfile(root, path)), required)));
        end

        function repositoryLandingPageIsActionable(testCase)
            root = repositoryRoot();
            readme = fileread(fullfile(root, 'README.rst'));
            requiredText = {'setup_ddpack', 'ddpack.solve', ...
                'run_publication_examples', 'run_all_tests', ...
                'run_benchmarks', 'DDPACK_Manual/DDPACK_Manual.pdf', ...
                'does not imply'};
            for index = 1:numel(requiredText)
                testCase.verifySubstring(readme, requiredText{index});
            end
        end

        function repositoryContainsOnlyApprovedMarkdown(testCase)
            root = repositoryRoot();
            markdown = dir(fullfile(root, '**', '*.md'));
            paths = string(fullfile({markdown.folder}, {markdown.name}));
            relativePaths = erase(paths, string(root) + string(filesep));
            generated = startsWith(replace(relativePaths, string(filesep), '/'), {'artifacts/','release/','release_audit/','build','audit/','reports/'});
            approved = "README.md";
            testCase.verifyEqual(sort(relativePaths(~generated)), sort(approved));
        end

        function publicDocumentationStatesMaintainedScope(testCase)
            root = repositoryRoot();
            text = fileread(fullfile(root,'README.rst'));
            testCase.verifySubstring(text,'Implementation scope and differences');
            testCase.verifySubstring(text,'package-specific implementation variants');
            testCase.verifySubstring(text,'https://github.com/FR-Xia/DDPACK/releases/tag/v1.1.0');
            testCase.verifySubstring(text,'BSD-3-Clause');
        end

        function manualCommandsAndAlgorithmsAreCurrent(testCase)
            root = repositoryRoot();
            manual = fileread(fullfile(root, 'DDPACK_Manual', 'DDPACK_Manual.tex'));
            requiredText = {'setup_ddpack', 'run_publication_examples', ...
                'run_all_tests', 'run_benchmarks', 'free_time_pddp', ...
                'waypoint_pddp', 'terminal_defect_pddp', ...
                'geometric_safety_pddp_2d', 'geometric_safety_pddp_3d', ...
                'penalty_free_time_ddp'};
            for index = 1:numel(requiredText)
                testCase.verifySubstring(manual, requiredText{index});
            end
            testCase.verifyFalse(contains(lower(manual), 'admm'));
        end

        function formalExamplesAreNamedInRunner(testCase)
            root = repositoryRoot();
            runner = fileread(fullfile(root, 'matlab', 'examples', 'run_publication_examples.m'));
            names = {'example_planar_point_mass_trajectory', ...
                'example_terminal_defect_trajectory', ...
                'example_geometric_safety_pddp_2d_obstacle_avoidance', ...
                'example_geometric_safety_pddp_3d_obstacle_avoidance', ...
                'example_penalty_free_time_reactor_regulation', ...
                'example_thermal_regulation', ...
                'example_energy_storage_management'};
            for index = 1:numel(names)
                testCase.verifySubstring(runner, names{index});
            end
        end

        function namingAndScopeAuditPasses(testCase)
            report = validate_naming_and_scope();
            testCase.verifyTrue(report.success);
            testCase.verifyEqual(report.status, 'PASS');
            testCase.verifyEqual(report.generality_level, 'G2_LIMITED');
        end

        function citationAndCodeMetadataAreConsistent(testCase)
            root = repositoryRoot();
            metadata = jsondecode(fileread(fullfile(root,'codemeta.json')));
            testCase.verifyEqual(metadata.version,ddpack.version());
            testCase.verifyEqual(metadata.codeRepository,'https://github.com/FR-Xia/DDPACK');
            testCase.verifySubstring(metadata.license,'BSD-3-Clause');
        end

        function versionsAreConsistent(testCase)
            root = repositoryRoot();
            citation = fileread(fullfile(root,'CITATION.cff'));
            testCase.verifySubstring(citation,['version: ' ddpack.version()]);
            cmake = fileread(fullfile(root,'CMakeLists.txt'));
            testCase.verifySubstring(cmake,['VERSION ' ddpack.version()]);
        end

        function formalCodeContainsNoPrivateAbsolutePath(testCase)
            root = repositoryRoot();
            folders = {fullfile(root, 'matlab', 'src'), fullfile(root, 'matlab', 'examples'), ...
                fullfile(root, 'matlab', 'tests'), fullfile(root, 'matlab', 'benchmarks'), ...
                fullfile(root, 'matlab', 'tools')};
            windowsUserPattern = ['[A-Za-z]:' '\\Users\\'];
            unixHomePattern = ['/' 'home/' '[^/]+' '/'];
            macUserPattern = ['/' 'Users/' '[^/]+' '/'];
            privatePathPattern = [windowsUserPattern '|' ...
                unixHomePattern '|' macUserPattern];
            for folderIndex = 1:numel(folders)
                files = dir(fullfile(folders{folderIndex}, '**', '*'));
                files = files(~[files.isdir]);
                for fileIndex = 1:numel(files)
                    [~, ~, extension] = fileparts(files(fileIndex).name);
                    if ~ismember(lower(extension), {'.m', '.ps1'}), continue; end
                    content = fileread(fullfile(files(fileIndex).folder, files(fileIndex).name));
                    testCase.verifyEmpty(regexp(content, ...
                        privatePathPattern, 'once'), ...
                        sprintf('Private path in %s', files(fileIndex).name));
                end
            end
        end

        function publicSourceExcludesPrivateSubmissionFiles(testCase)
            root = repositoryRoot();
            for name={'HANDOFF.md','submission','manuscript','experiments/sensitivity_results'}
                testCase.verifyFalse(isfile(fullfile(root,name{1})) || isfolder(fullfile(root,name{1})));
            end
        end
    end
end

function root = repositoryRoot()
root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))); 
end
