function summary=run_derivative_audit(outputDir)
%RUN_DERIVATIVE_AUDIT Nine formal models, three points, three stencil widths.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab','tools'));setup_ddpack;
rngState=rng;rngCleanup=onCleanup(@()rng(rngState)); %#ok<NASGU>
if isfolder(outputDir)||isfile(outputDir)
    error('ddpack:derivatives:ExistingOutput','Choose a fresh derivative audit directory.');
end
mkdir(outputDir);
names={'vibration','planar','terminal_defect','obstacle_2d','obstacle_3d','reactor','pendulum','thermal','storage'};
widths=[1e-4,1e-5,1e-6];reports=cell(0,1);rows=cell(0,10);
for index=1:numel(names)
    [problem,options]=comparison_case(names{index});
    for sample=1:3
        state=(1-.25*sample)*problem.x0+.25*sample*problem.xf+ ...
            .07*sin((1:numel(problem.x0)).'+sample);
        control=.13*sample*problem.u_max(:).*cos((1:numel(problem.u_max)).'+2*sample);
        for width=widths
            report=ddpack.checkDerivatives(problem,state,control,'Algorithm',options.Algorithm, ...
                'DifferenceStep',width,'TimeScale',1.3+.4*sample,'IntegrationStep',.037);
            report.case=names{index};report.sample=sample;reports{end+1,1}=report; %#ok<AGROW>
            for check=report.checks
                rows(end+1,:)={names{index},sample,width,check.name,check.passed, ...
                    check.max_absolute_error,check.max_scaled_error,check.max_tolerance_ratio,check.category,check.message}; %#ok<AGROW>
            end
        end
    end
end
tableData=cell2table(rows,'VariableNames',{'case','sample','difference_step','check','passed', ...
    'max_absolute_error','max_scaled_error','max_tolerance_ratio','category','message'});
writetable(tableData,fullfile(outputDir,'derivative_checks.csv'));
summary=struct('success',all(tableData.passed),'formal_cases',numel(names), ...
    'probe_points_per_case',3,'stencil_widths',widths,'reports',numel(reports), ...
    'checks',height(tableData),'failed_checks',sum(~tableData.passed), ...
    'matlab_version',version,'maximum_tolerance_ratio',max(tableData.max_tolerance_ratio));
writeJson(fullfile(outputDir,'reports.json'),reports);writeJson(fullfile(outputDir,'summary.json'),summary);
assert(summary.success,'ddpack:derivatives:AuditFailed','Derivative mismatches: %s',outputDir);
end
function writeJson(path,value)
fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
