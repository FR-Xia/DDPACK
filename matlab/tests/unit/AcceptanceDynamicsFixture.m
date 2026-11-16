classdef AcceptanceDynamicsFixture < Dynamics
    % Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    properties
        Mode
    end
    methods
        function obj=AcceptanceDynamicsFixture(mode),obj.Mode=mode;end
        function value=F(obj,x,~,varargin)
            if x>.5
                switch obj.Mode
                    case 'throws',error('ddpack:test:DynamicsCause','Late dynamics callback failure');
                    case 'nan',value=NaN;return;
                    case 'complex',value=1i;return;
                    case 'shape',value=[.4;.4];return;
                end
            end
            value=.4;
        end
        function v=Phi(~,~,~,varargin),v=1;end
        function v=beta(~,~,~,varargin),v=0;end
    end
end
