function save(solution, filename)
%SAVE Save a normalized DDPACK solution with schema metadata.
if ~isstruct(solution) || ~isfield(solution, 'metadata') || ...
        ~isfield(solution.metadata, 'schema_version')
    error('ddpack:save:InvalidSolution', ...
        'Input is not a normalized DDPACK solution.');
end
folder = fileparts(filename);
if ~isempty(folder) && ~isfolder(folder)
    mkdir(folder);
end
payload = struct('solution', solution, 'file_format', 'DDPACK_SOLUTION', ...
    'schema_version', 1, 'ddpack_version', ddpack.version());
builtin('save', filename, '-struct', 'payload');
if ~isfile(filename)
    error('ddpack:save:WriteFailed', 'Solution file was not created: %s', filename);
end
end
