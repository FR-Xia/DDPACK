classdef AcceptanceConstraintFixture < Constraints
    % Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    properties
        Mode
    end
    methods
        function obj=AcceptanceConstraintFixture(mode),obj.Mode=mode;end
        function value=G(obj,x,u,~)
            switch obj.Mode
                case 'nan',value=NaN;
                case 'inf',value=Inf;
                case 'complex',value=1i;
                case 'empty',value=[];
                case 'matrix',value=ones(2);
                case 'changing',value=ones(1,1+(x(1)>0));
                case 'throws',error('ddpack:test:ConstraintCause','Constraint callback error');
                case 'held_control',value=u(1);
                otherwise,value=[x(1);1];
            end
        end
        function v=G_x(~,x,~,~),v=zeros(1,numel(x));end
        function v=G_u(~,~,u,~),v=zeros(1,numel(u));end
    end
end
