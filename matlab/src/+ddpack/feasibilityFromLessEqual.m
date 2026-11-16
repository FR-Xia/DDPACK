function varargout = feasibilityFromLessEqual(varargin)
%FEASIBILITYFROMLESSEQUAL Convert c <= 0 data to DDPACK's G >= 0 convention.
%   G = DDPACK.FEASIBILITYFROMLESSEQUAL(C) returns G = -C.
%   [G,GX,GU] = DDPACK.FEASIBILITYFROMLESSEQUAL(C,CX,CU) also negates
%   derivatives, preserving a consistent constraint representation.
varargout = cell(size(varargin));
for index = 1:numel(varargin)
    validateattributes(varargin{index}, {'numeric'}, {'real'});
    varargout{index} = -varargin{index};
end
end
