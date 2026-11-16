function results = run_all_tests(varargin)
%RUN_ALL_TESTS Repository-level test entry point.
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
testsRoot = fullfile(repoRoot, 'matlab', 'tests');
addpath(testsRoot);
results = execute_test_suite(varargin{:});
end
