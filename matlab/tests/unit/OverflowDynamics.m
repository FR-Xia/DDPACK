classdef OverflowDynamics < PointMassDynamics
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods
        function obj=OverflowDynamics()
            obj@PointMassDynamics(zeros(2),[1;0],zeros(2,1));
        end
        function value=F(obj,x,u,varargin)
            value=F@PointMassDynamics(obj,x,u,varargin{:});
            if any(abs(u)>0),value(:)=Inf;end
        end
    end
end
