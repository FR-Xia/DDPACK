function run_language_comparison(outputDir)
%RUN_LANGUAGE_COMPARISON Paired nine-case protocol; 3 warmups, 5 measured runs.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab','tools')); setup_ddpack;
assert(~isfolder(outputDir), 'Choose a new comparison directory.'); mkdir(outputDir);
names = {'vibration','planar','terminal_defect','obstacle_2d','obstacle_3d', ...
    'reactor','pendulum','thermal','storage'};
rows = cell(0,5);
for index = 1:numel(names)
    name = names{index}; [problem, options] = comparison_case(name); state = rng;
    for warm = 1:3
        rng(state); evalc('solution = ddpack.solve(problem, options);');
    end
    target = fullfile(outputDir,name); mkdir(target);
    for repeat = 1:5
        rng(state); started = tic;
        logText = evalc('solution = ddpack.solve(problem, options);');
        seconds = toc(started);
        assert(solution.success, solution.message);
        metric = solution.metadata.metrics;
        rows(end+1,:) = {name, repeat, seconds, solution.success, metric.returned_objective}; %#ok<AGROW>
        if repeat == 1
            write_comparison_parameters(problem, options, target);
            writematrix(solution.states,fullfile(target,'states.csv'));
            writematrix(solution.controls,fullfile(target,'controls.csv'));
            writematrix(solution.time,fullfile(target,'time.csv'));
            if isfield(solution.raw,'line_search')
                writematrix(solution.raw.line_search.trials,fullfile(target,'line_search_trials.csv'));
                writematrix(solution.raw.M,fullfile(target,'merit.csv'));
            end
            metric.final_time = solution.time(end);
            if isfield(solution.raw,'geometric_refinement')
                metric.geometric_refinement=solution.raw.geometric_refinement;
            end
            fid = fopen(fullfile(target,'metrics.json'),'w');
            fprintf(fid,'%s',jsonencode(metric,'PrettyPrint',true)); fclose(fid);
            save(fullfile(target,'configuration.mat'),'problem','options','state');
            fid = fopen(fullfile(target,'solver.txt'),'w'); fprintf(fid,'%s',logText); fclose(fid);
        end
    end
    fprintf('%s MATLAB comparison complete\n',name);
end
writetable(cell2table(rows,'VariableNames',{'case','repeat','seconds','success','returned_cost'}),fullfile(outputDir,'timings.csv'));
fid = fopen(fullfile(outputDir,'environment.txt'),'w');
fprintf(fid,'MATLAB %s\nArchitecture %s\nThreads %d\n',version,computer,maxNumCompThreads);
fprintf(fid,'3 warmups + 5 measured solves per case; RNG restored before each solve.\n');
fprintf(fid,'Timed: public solve, validation, diagnostics, captured MATLAB console output.\n');
fprintf(fid,'Excluded: setup, RNG reset, plots, file output, MATLAB startup and native compilation.\n');
fclose(fid);
end
