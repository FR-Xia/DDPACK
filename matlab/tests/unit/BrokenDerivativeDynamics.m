classdef BrokenDerivativeDynamics < PointMassDynamics
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    properties
        Mode
    end
    methods
        function obj=BrokenDerivativeDynamics(mode)
            obj@PointMassDynamics([0,1;-1,-.4],[0;.2],[0;0]);obj.Mode=mode;
        end
        function transition=Phi(obj,x,u,varargin)
            transition=Phi@PointMassDynamics(obj,x,u,varargin{:});
            switch obj.Mode
                case 'wrong',transition=transition+.1*eye(2);
                case 'shape',transition=zeros(1,2);
                case 'nonfinite',transition(:)=NaN;
                case 'throw',error('ddpack:test:DerivativeCallback','Deliberate callback failure.');
            end
        end
        function y=F(obj,x,u,varargin)
            y=F@PointMassDynamics(obj,x,u,varargin{:});
            if strcmp(obj.Mode,'value')&&isequal(x,[.2;-.3])&&isequal(u,.1)&& ...
                    ~isempty(varargin)&&isequal(varargin{1},1.7),y(:)=NaN;end
        end
    end
end
