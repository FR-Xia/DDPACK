function solution = load(filename)
%LOAD Load and validate a normalized DDPACK solution file.
if ~isfile(filename)
    error('ddpack:load:MissingFile', 'Solution file does not exist: %s', filename);
end
payload = builtin('load', filename, 'solution', 'file_format', 'schema_version');
required = {'solution', 'file_format', 'schema_version'};
for index = 1:numel(required)
    if ~isfield(payload, required{index})
        error('ddpack:load:InvalidFormat', ...
            'Missing field in solution file: %s', required{index});
    end
end
if ~strcmp(payload.file_format, 'DDPACK_SOLUTION') || payload.schema_version ~= 1
    error('ddpack:load:UnsupportedVersion', ...
        'Unsupported DDPACK solution file format or schema version.');
end
solution = payload.solution;
end
