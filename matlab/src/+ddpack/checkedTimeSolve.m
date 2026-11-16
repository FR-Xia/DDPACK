function solution = checkedTimeSolve(hessian,rhs)
%CHECKEDTIMESOLVE Reject singular or nonfinite scalar time curvature.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
if ~isscalar(hessian) || ~isreal(hessian) || ~isreal(rhs) || ...
        ~isfinite(hessian) || any(~isfinite(rhs(:))) || abs(hessian)<1e-18
    error('ddpack:numerics:SingularTimeHessian','Time Hessian is singular or nonfinite.');
end
solution=hessian\rhs;
if any(~isfinite(solution(:)))
    error('ddpack:numerics:SingularTimeHessian','Time solve produced a nonfinite update.');
end
end
