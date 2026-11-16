function summary=profile_solver_cases(outputDir)
%PROFILE_SOLVER_CASES Profile two warmed formal solvers without timing claims.
% Profiler timings include instrumentation overhead. Use the separate paired
% public-solve protocol for performance measurements.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab','tools'));setup_ddpack;
if isfolder(outputDir)||isfile(outputDir)
    error('ddpack:profile:ExistingOutput','Choose a fresh profiler output directory.');
end
mkdir(outputDir);originalRng=rng;rngCleanup=onCleanup(@()rng(originalRng)); %#ok<NASGU>
profileCleanup=onCleanup(@()profile('off')); %#ok<NASGU>
names={'planar','pendulum'};rows=cell(0,4);
for index=1:numel(names)
    [problem,options]=comparison_case(names{index});state=rng;
    for warm=1:3,rng(state);evalc('solution=ddpack.solve(problem,options);');end
    profile clear;profile on;
    rng(state);evalc('solution=ddpack.solve(problem,options);');
    profile off;assert(solution.success,solution.message);
    stats=profile('info');save(fullfile(outputDir,[names{index} '_profile.mat']),'stats');
    functions=stats.FunctionTable;
    tableData=table(string({functions.FunctionName}).',[functions.TotalTime].', ...
        [functions.NumCalls].','VariableNames',{'function','seconds','calls'});
    tableData=sortrows(tableData,'seconds','descend');
    writetable(tableData,fullfile(outputDir,[names{index} '_functions.csv']));
    callbackCalls=0;
    for f=1:numel(functions)
        if startsWith(functions(f).FunctionName,'waypoint_pddp>@(v)')
            callbackCalls=callbackCalls+functions(f).NumCalls;
        elseif strcmp(functions(f).FunctionName,'waypoint_pddp')
            writematrix(functions(f).ExecutedLines,fullfile(outputDir,[names{index} '_waypoint_lines.csv']));
        end
    end
    rows(end+1,:)={names{index},solution.iterations,callbackCalls,solution.success}; %#ok<AGROW>
end
tableData=cell2table(rows,'VariableNames',{'case','outer_iterations','trajectory_cell_checks','success'});
writetable(tableData,fullfile(outputDir,'profile_summary.csv'));
summary=struct('success',all(tableData.success),'cases',numel(names), ...
    'purpose','Locate redundant work; profiled wall times are not benchmark timings.');
fid=fopen(fullfile(outputDir,'summary.json'),'w');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
end
