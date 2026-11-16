function [identity, wasLegacy] = resolveAlgorithm(name, varargin)
%RESOLVEALGORITHM Resolve a public algorithm ID or deprecated legacy alias.
parser = inputParser;
addParameter(parser, 'Warn', false, @(x) islogical(x) && isscalar(x));
parse(parser, varargin{:});

if ~(ischar(name) || (isstring(name) && isscalar(name)))
    error('ddpack:algorithm:InvalidName', ...
        'Algorithm name must be scalar text.');
end
requested = lower(char(name));
registry = ddpack.algorithmRegistry();
identity = struct([]);
wasLegacy = false;
for index = 1:numel(registry)
    if strcmp(requested, registry(index).algorithm_id)
        identity = registry(index);
        break
    end
    if any(strcmp(requested, lower(string(registry(index).legacy_names))))
        identity = registry(index);
        wasLegacy = true;
        break
    end
end
if isempty(identity)
    error('ddpack:algorithm:UnknownName', ...
        'Unknown DDPACK algorithm name: %s', requested);
end
if parser.Results.Warn && wasLegacy
    warning('ddpack:DeprecatedAlgorithmName', ...
        ['Algorithm name ''%s'' is deprecated; use ''%s'' (%s). ' ...
        'The legacy name does not imply a strict reference reproduction.'], ...
        requested, identity.algorithm_id, identity.public_name);
end
end
