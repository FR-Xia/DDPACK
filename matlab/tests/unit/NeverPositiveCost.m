classdef NeverPositiveCost < QuadraticCost
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    methods
        function obj=NeverPositiveCost()
            obj@QuadraticCost(eye(2),eye(2),1);
        end
        function value=L_uu(~,~,u,varargin)
            value=-1e100*eye(numel(u));
        end
    end
end
