function raw = finalizeTrajectoryCost(raw, cost, target)
%FINALIZETRAJECTORYCOST Evaluate the returned trajectory independently of J.
% Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
x = raw.x; u = raw.u;
value = cost.phi(x{end}, target);
for k = 1:numel(x)-1
    value = value + cost.L(x{k}, target, u{k}, 1, raw.t(k+1)-raw.t(k));
end
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('ddpack:numerics:NonfiniteTrajectory', 'Returned trajectory objective is not a finite real scalar.');
end
raw.returned_cost = value;
raw.initial_cost = raw.J(1);
end
