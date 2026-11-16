Constraint and dynamics acceptance
==================================

MATLAB evaluates constraints at every state node, including the terminal node
using the last active held control. The stage contract is
``G(x,u,dt)=dt*g(x,u)``; acceptance uses ``G(x,u,1)`` and an unweighted constraint
tolerance. Different physical units still require model-appropriate scaling.
Nonempty constraints must return finite real numeric vectors with fixed widths.
Invalid outputs or callback exceptions cannot become successful acceptance;
wrapped exceptions retain their original cause.

Euler and four-substep RK4 diagnostics must complete every interval. Their
``complete``, evaluated/expected interval counts and failure step/identifier/
message are retained. Missing defects stay NaN; an incomplete maximum is Inf.
Incomplete diagnostics fail acceptance even with an infinite RK4 tolerance.

These local checks do not certify full continuous-model endpoints. Pendulum
and thermal full replays can exceed their endpoint tolerances. The supported
geometric models additionally require analytic complete-rollout endpoint and
clearance checks. See ``VALIDATION.rst`` for tests and reproduction.
