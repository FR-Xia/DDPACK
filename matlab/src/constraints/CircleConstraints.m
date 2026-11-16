% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause

% Class for obstacle
classdef CircleConstraints < Constraints
    properties
        Center; % center of the obstacle
        r; % radius of the obstacle
        SafetyMargin = 0; % additional physical clearance
    end
    
    methods
        % Constructor
        function obj = CircleConstraints(Center, r, safetyMargin)
            obj.Center = Center;
            obj.r = r;
            if nargin >= 3, obj.SafetyMargin = safetyMargin; end
        end
        
        % Circle constraints
        function G = G(obj, x, ~, dt)
            safeRadius = obj.r + obj.SafetyMargin;
            G = ((x(1:2, 1)' - obj.Center) * ...
                (x(1:2, 1)' - obj.Center)' - safeRadius^2) .* dt;
        end
        
        % Circle constraints derivatives
        function G_x = G_x(obj, x, ~, dt)
            G_x = [2 .* (x(1:2, 1)' - obj.Center), 0] .* dt;
        end
        function G_u = G_u(~, ~, u, ~)
            G_u = zeros(1, numel(u));
        end
    end
end

