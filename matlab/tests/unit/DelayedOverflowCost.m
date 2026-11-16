classdef DelayedOverflowCost < QuadraticCost
    % Overflow after an earlier finite running-cost term in the same rollout.
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    properties(SetAccess=private)
        RequireControl=true
    end
    methods
        function obj=DelayedOverflowCost(requireControl)
            obj@QuadraticCost(eye(2),eye(2),1);
            if nargin>0,obj.RequireControl=requireControl;end
        end
        function value=L(obj,x,target,u,varargin)
            value=L@QuadraticCost(obj,x,target,u,varargin{:});
            if x(1)>.5 && (~obj.RequireControl || any(abs(u)>0)),value=Inf;end
        end
    end
end
