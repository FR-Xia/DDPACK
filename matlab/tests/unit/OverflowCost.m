classdef OverflowCost < QuadraticCost
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods
        function obj=OverflowCost()
            obj@QuadraticCost(eye(2),eye(2),1);
        end
        function value=L(obj,x,target,u,varargin)
            value=L@QuadraticCost(obj,x,target,u,varargin{:});
            if any(abs(u)>0),value=Inf;end
        end
    end
end
