function repoRoot = setup_ddpack(varargin)
%SETUP_DDPACK Add DDPACK source directories to the MATLAB path.
%   ROOT = SETUP_DDPACK() locates the repository from this file, adds the
%   formal source tree, and returns the repository root. The function does
%   not change the current folder and does not persist the MATLAB path.
%
%   ROOT = SETUP_DDPACK('IncludeExamples', true) also adds example folders.
%   Set IncludeBenchmarks to add the reproducible benchmark entry point.

parser = inputParser;
parser.FunctionName = 'setup_ddpack';
addParameter(parser, 'IncludeExamples', false, ...
    @(x) islogical(x) && isscalar(x));
addParameter(parser, 'IncludeBenchmarks', false, ...
    @(x) islogical(x) && isscalar(x));
parse(parser, varargin{:});

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
srcRoot = fullfile(repoRoot, 'matlab', 'src');
if ~isfolder(srcRoot)
    error('ddpack:setup:MissingSource', ...
        'DDPACK source directory was not found: %s', srcRoot);
end

addpath(genpath(srcRoot));
if parser.Results.IncludeExamples
    addpath(genpath(fullfile(repoRoot, 'matlab', 'examples')));
end
if parser.Results.IncludeBenchmarks
    addpath(fullfile(repoRoot, 'matlab', 'benchmarks'));
end
end
