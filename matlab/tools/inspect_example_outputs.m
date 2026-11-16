function report = inspect_example_outputs(runDir)
%INSPECT_EXAMPLE_OUTPUTS Validate one generated example run.
if ~isfolder(runDir)
    error('ddpack:inspect:MissingRun', 'Example run does not exist: %s', runDir);
end
entries = dir(runDir); entries = entries([entries.isdir]);
entries = entries(~ismember({entries.name}, {'.', '..'}));
required = {'config.json', 'metrics.json', 'metrics.csv', 'solution.mat', ...
    'run.log', 'summary.txt', 'trajectory.png', 'trajectory.pdf', 'environment.txt', ...
    'defect_diagnostics.json', 'defect_diagnostics.csv', ...
    'defect_diagnostics.png', 'defect_diagnostics.pdf'};
rows = cell(0, 4); allPassed = true;
for index = 1:numel(entries)
    folder = fullfile(entries(index).folder, entries(index).name);
    passed = true; message = 'validated';
    try
        for fileIndex = 1:numel(required)
            info = dir(fullfile(folder, required{fileIndex}));
            assert(~isempty(info) && info.bytes > 0, 'Missing or empty: %s', required{fileIndex});
        end
        config = jsondecode(fileread(fullfile(folder, 'config.json'))); %#ok<NASGU>
        metrics = jsondecode(fileread(fullfile(folder, 'metrics.json')));
        numericValues = struct2cell(metrics);
        for valueIndex = 1:numel(numericValues)
            value = numericValues{valueIndex};
            if isnumeric(value), assert(all(isfinite(value(:))), 'Non-finite metric.'); end
        end
        imageData = imread(fullfile(folder, 'trajectory.png'));
        assert(numel(imageData) > 1000 && std(double(imageData(:))) > 1, 'Blank PNG.');
        defectImage = imread(fullfile(folder, 'defect_diagnostics.png'));
        assert(numel(defectImage) > 1000 && std(double(defectImage(:))) > 1, ...
            'Blank defect-diagnostics PNG.');
        solution = ddpack.load(fullfile(folder, 'solution.mat'));
        assert(size(solution.states, 2) >= 10, 'Trajectory has too few samples.');
        assert(norm(solution.states(:, 1) - solution.metadata.problem.x0) < 1e-10, ...
            'Initial state mismatch.');
        logText = lower(fileread(fullfile(folder, 'run.log')));
        assert(isempty(regexp(logText, 'undefined function|singular matrix|\<nan\>|\<inf\>', 'once')), ...
            'Failure keyword in run log.');
    catch exception
        passed = false; message = exception.message;
    end
    allPassed = allPassed && passed;
    rows(end+1, :) = {entries(index).name, passed, message, ...
        fullfile(folder, 'trajectory.png')}; %#ok<AGROW>
end
report = struct('success', allPassed, 'run_dir', runDir, 'rows', {rows});
if ~allPassed
    error('ddpack:inspect:Failed', 'One or more example artifact bundles failed inspection.');
end
end
