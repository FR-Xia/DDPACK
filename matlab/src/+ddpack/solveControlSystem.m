function [solution,hessian,diagnostics] = solveControlSystem(hessian,rhs,diagnostics)
%SOLVECONTROLSYSTEM Finite, conditioned control solve with bounded damping.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
if nargin<3
    diagnostics=struct('linear_solves',0,'regularized_solves',0, ...
        'maximum_damping',0,'backward_retries',0);
end
if ~isreal(hessian) || ~isreal(rhs) || any(~isfinite(hessian(:))) || any(~isfinite(rhs(:)))
    error('ddpack:numerics:NonfiniteLinearSystem','Control Hessian or right-hand side is nonfinite.');
end
if isempty(hessian) || size(hessian,1)~=size(hessian,2) || size(rhs,1)~=size(hessian,1)
    error('ddpack:numerics:LinearSystemDimensions','Control system dimensions are inconsistent.');
end
base=(hessian+hessian.')/2;scale=max(1,norm(base,Inf));
if any(~isfinite(base(:))) || ~isfinite(scale)
    error('ddpack:numerics:NonfiniteLinearSystem','Control system scale overflowed.');
end
for attempt=0:11
    damping=0;if attempt>0,damping=scale*10^(attempt-9);end
    candidate=base+damping*eye(size(base));
    if any(~isfinite(candidate(:))),continue;end
    [~,flag]=chol(candidate);
    if flag==0 && rcond(candidate)>=1e-12
        solution=candidate\rhs;
        residual=norm(candidate*solution-rhs,Inf);
        if all(isfinite(solution(:))) && residual<=1e-10*max(1,norm(candidate,Inf)*norm(solution,Inf)+norm(rhs,Inf))
            hessian=candidate;
            diagnostics.linear_solves=diagnostics.linear_solves+1;
            diagnostics.regularized_solves=diagnostics.regularized_solves+double(damping>0);
            diagnostics.maximum_damping=max(diagnostics.maximum_damping,damping);
            return;
        end
    end
end
error('ddpack:numerics:RegularizationExhausted','Control solve failed after 12 bounded damping attempts.');
end
