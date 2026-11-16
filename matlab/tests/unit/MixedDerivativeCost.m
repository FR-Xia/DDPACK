classdef MixedDerivativeCost < QuadraticCost_fft
    % Nonseparable fixed-time cost to exercise nonzero mixed derivatives.
    % Copyright (c) 2026 Feiran Xia
    % Beijing Institute of Technology, China
    % SPDX-License-Identifier: BSD-3-Clause
    properties
        S=[.2,-.3;.4,.1]
    end
    methods
        function obj=MixedDerivativeCost()
            obj@QuadraticCost_fft(eye(2),3*eye(2),2*eye(2));
        end
        function y=L(obj,x,target,u,h)
            y=L@QuadraticCost_fft(obj,x,target,u,h)+h*(x-target).'*obj.S*u;
        end
        function y=L_x(obj,x,target,u,h)
            y=L_x@QuadraticCost_fft(obj,x,target,u,h)+h*obj.S*u;
        end
        function y=L_u(obj,x,u,h)
            y=L_u@QuadraticCost_fft(obj,x,u,h)+h*obj.S.'*x;
        end
        function y=L_xu(obj,~,~,h),y=h*obj.S;end
        function y=L_ux(obj,~,~,h),y=h*obj.S.';end
    end
end
