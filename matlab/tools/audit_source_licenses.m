function report = audit_source_licenses(varargin)
%AUDIT_SOURCE_LICENSES Inventory project sources and their declared license.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
parser = inputParser;
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
parse(parser, varargin{:});
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
patterns = {fullfile('matlab','**','*.m'), ...
    fullfile('matlab','**','*.ps1'), fullfile('cpp','**','*.cpp'), ...
    fullfile('cpp','**','*.hpp'), fullfile('cpp','**','*.cmake'), ...
    fullfile('cpp','**','*.cmake.in'), fullfile('cpp','**','CMakeLists.txt'), ...
    fullfile('experiments','*.py'), ...
    'CMakeLists.txt'};
paths = strings(0,1);
for k = 1:numel(patterns)
    files = dir(fullfile(repoRoot, patterns{k}));
    for j = 1:numel(files)
        paths(end+1,1) = string(fullfile(files(j).folder,files(j).name)); %#ok<AGROW>
    end
end
paths = sort(unique(paths));
licenseText = fileread(fullfile(repoRoot,'LICENSE.txt'));
licenseTextValid = contains(licenseText,'BSD 3-Clause License') && ...
    contains(licenseText,'Copyright (c) 2026, Feiran Xia') && ...
    contains(licenseText,'Redistribution and use in source and binary forms');
licenseCopiesMatch = strcmp(licenseText,fileread(fullfile(repoRoot,'Licence.txt')));
rows = cell(0,10);
attributed = 0;
for k = 1:numel(paths)
    filename = char(paths(k));
    relative = replace(erase(paths(k),string(repoRoot)+filesep),filesep,'/');
    content = fileread(filename);
    % Only inspect actual comment notices, not quoted strings in audit tools.
    notices = regexp(content, ...
        '(?mi)^\s*(?:%|//|#)\s*Copyright\s*(?:\(c\)|\xA9|\d)[^\r\n]*', 'match');
    spdx = regexp(content, ...
        '(?mi)^\s*(?:%|//|#)\s*SPDX-License-Identifier:\s*([^\r\n]+)', 'tokens');
    unknown = any(~contains(string(notices),'Feiran Xia'));
    for j = 1:numel(spdx)
        unknown = unknown || ~strcmp(strtrim(spdx{j}{1}),'BSD-3-Clause');
    end
    if isempty(notices)
        evidence = 'Project LICENSE.txt';
    else
        attributed = attributed+1;
        evidence = char(strjoin(strtrim(string(notices)),' | '));
    end
    if unknown
        permission = 'REVIEW_FILE_NOTICE';
    else
        permission = 'BSD-3-Clause';
    end
    relation = 'DDPACK project implementation';
    if relative == "cpp/src/solver.cpp"
        relation = 'Native implementation of MATLAB algorithm branches';
    elseif relative == "cpp/src/models.cpp"
        relation = 'Native implementation of MATLAB dynamics and costs';
    elseif startsWith(relative,"matlab/tools/") || startsWith(relative,"experiments/")
        relation = 'Project validation, analysis, and release tools';
    elseif contains(relative,"/tests/") || contains(relative,"/examples/")
        relation = 'Project tests and examples';
    end
    rows(end+1,:) = {char(relative),evidence,'BSD-3-Clause (LICENSE.txt)', ...
        relation,permission,unknown,'Feiran Xia', ...
        'Beijing Institute of Technology',sha256File(filename),'project'}; %#ok<AGROW>
end
tableOut = cell2table(rows,'VariableNames',{'file','source_evidence', ...
    'original_license_evidence','current_use','redistributable', ...
    'manual_confirmation_required','copyright_holder','affiliation', ...
    'sha256','component_scope'});
pending = sum(tableOut.manual_confirmation_required);
report = struct('success',licenseTextValid && licenseCopiesMatch && pending==0, ...
    'license_identifier','BSD-3-Clause','copyright_holder','Feiran Xia', ...
    'affiliation','Beijing Institute of Technology', ...
    'license_text_valid',licenseTextValid,'license_copies_match',licenseCopiesMatch, ...
    'audited_file_count',height(tableOut),'attributed_file_count',attributed, ...
    'manual_confirmation_count',pending,'unreported_source_blockers',pending, ...
    'dependency_notice','docs/SOURCE_LICENSES.rst', ...
    'entries',table2struct(tableOut));
outputDir = char(parser.Results.OutputDir);
if ~isempty(outputDir)
    if ~isfolder(outputDir), mkdir(outputDir); end
    writetable(tableOut,fullfile(outputDir,'source_license_audit.csv'));
    writeText(fullfile(outputDir,'source_license_audit.json'), ...
        jsonencode(report,'PrettyPrint',true));
    writeText(fullfile(outputDir,'source_license_audit.txt'),sprintf([ ...
        'DDPACK source license inventory\nCopyright holder: Feiran Xia\n' ...
        'Affiliation: Beijing Institute of Technology\nLicense: BSD-3-Clause\n' ...
        'Files: %d\nFile notices requiring review: %d\n' ...
        'External dependencies: see docs/SOURCE_LICENSES.rst\n'],height(tableOut),pending));
end
end

function value = sha256File(filename)
fid = fopen(filename,'rb');
if fid<0, error('ddpack:audit:ReadFailed','Cannot read %s.',filename); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid,Inf,'*uint8');
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(bytes);
value = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end

function writeText(filename,value)
fid = fopen(filename,'w','n','UTF-8');
if fid<0, error('ddpack:audit:WriteFailed','Cannot write %s.',filename); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',value);
end
