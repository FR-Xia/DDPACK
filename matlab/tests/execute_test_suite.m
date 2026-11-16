function results = execute_test_suite(varargin)
%RUN_ALL_TESTS Run DDPACK tests and write machine-readable summaries.
parser = inputParser;
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
parse(parser, varargin{:});
testsRoot = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(testsRoot));
addpath(fullfile(repoRoot, 'matlab', 'tools'));
setup_ddpack('IncludeExamples', true);

outputDir = char(parser.Results.OutputDir);
if isempty(outputDir)
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(repoRoot, 'artifacts', 'test_runs', ['run_' stamp]);
end
if isfolder(outputDir)
    error('ddpack:tests:ExistingOutput', 'Test output directory already exists: %s', outputDir);
end
mkdir(outputDir);

started = tic;
results = runtests(testsRoot, 'IncludeSubfolders', true);
runtime = toc(started);
passed = sum([results.Passed]);
failed = sum([results.Failed]);
incomplete = sum([results.Incomplete]);
skipped = 0;
summary = struct('total', numel(results), 'passed', passed, ...
    'failed', failed, 'incomplete', incomplete, 'skipped', skipped, ...
    'runtime_seconds', runtime, ...
    'matlab_version', version, 'ddpack_version', ddpack.version());
writeText(fullfile(outputDir, 'test_summary.json'), jsonencode(summary, 'PrettyPrint', true));
writeText(fullfile(outputDir, 'test_summary.txt'), sprintf([ ...
    'DDPACK test summary\n\n- Total: %d\n- Passed: %d\n' ...
    '- Failed: %d\n- Incomplete: %d\n- Skipped: %d\n- Runtime: %.3f s\n'], ...
    summary.total, passed, failed, incomplete, skipped, runtime));
writeText(fullfile(outputDir, 'matlab_environment.txt'), ...
    sprintf('MATLAB: %s\nComputer: %s\nArchitecture: %s\n', version, computer, computer('arch')));
writeJUnit(fullfile(outputDir, 'test_results.xml'), results, runtime);
writeText(fullfile(outputDir, 'console.log'), evalc('disp(results)'));
if failed > 0 || incomplete > 0
    error('ddpack:tests:Failed', '%d tests failed and %d were incomplete.', failed, incomplete);
end
end

function writeJUnit(filename, results, runtime)
lines = {sprintf('<?xml version="1.0" encoding="UTF-8"?>'), ...
    sprintf('<testsuite name="DDPACK" tests="%d" failures="%d" time="%.6f">', ...
    numel(results), sum([results.Failed]), runtime)};
for index = 1:numel(results)
    lines{end+1} = sprintf('  <testcase name="%s" time="%.6f">', ...
        xmlEscape(results(index).Name), results(index).Duration); %#ok<AGROW>
    if results(index).Failed
        lines{end+1} = '    <failure message="MATLAB test failure" />'; %#ok<AGROW>
    end
    lines{end+1} = '  </testcase>'; %#ok<AGROW>
end
lines{end+1} = '</testsuite>';
writeText(filename, strjoin(lines, newline));
end

function value = xmlEscape(value)
value = strrep(value, '&', '&amp;'); value = strrep(value, '"', '&quot;');
value = strrep(value, '<', '&lt;'); value = strrep(value, '>', '&gt;');
end

function writeText(filename, value)
fileId = fopen(filename, 'w');
if fileId < 0, error('ddpack:tests:WriteFailed', 'Cannot write %s.', filename); end
cleanup = onCleanup(@() fclose(fileId)); fprintf(fileId, '%s', value);
end
