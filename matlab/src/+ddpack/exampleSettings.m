function settings = exampleSettings(exampleName, varargin)
%EXAMPLESETTINGS Parse common settings for one reproducible example.
parser = inputParser;
parser.FunctionName = char(exampleName);
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Visible', true, @(x) islogical(x) && isscalar(x));
addParameter(parser, 'QuickMode', false, @(x) islogical(x) && isscalar(x));
parse(parser, varargin{:});
settings = parser.Results;
if strlength(string(settings.OutputDir)) == 0
    packageRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    settings.OutputDir = fullfile(fileparts(packageRoot), 'artifacts', 'example_runs', ...
        [char(exampleName) '_' stamp]);
end
settings.OutputDir = char(settings.OutputDir);
if isfolder(settings.OutputDir) || isfile(settings.OutputDir)
    error('ddpack:examples:ExistingOutput', ...
        'Refusing to overwrite existing example output: %s', settings.OutputDir);
end
end
