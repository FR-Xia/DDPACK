Returned objectives, feasible refinement and final-time bounds
==============================================================

Returned trajectory versus iteration history
--------------------------------------------

MATLAB free-time and waypoint PDDP evaluate the complete returned trajectory's
objective before returning, including an iteration-budget exit. The public
``objective`` and ``metadata.metrics.returned_objective`` use ``raw.returned_cost``;
``J``, trial and accepted histories retain their solver-native iteration
records. A final forward update is not inserted into an iteration history as
if another optimizer iteration had occurred. Therefore a budget-exit objective
can differ from the final history entry, while matching the returned trajectory.

The running-cost contract is time weighted. The returned value is terminal
cost plus ``cost.L(x_k,xf,u_k,1,t_{k+1}-t_k)`` over active controls. Physical time
vectors are supplied on initial/failure exits as well. If the final update has
a nonfinite objective, MATLAB returns the last complete finite nominal with a
numerical-failure status. Callback programming errors still propagate. The
cost-decrease acceptance flag compares the returned objective with the original
initial objective, rather than comparing two history entries. Native C++ already
recomputes its returned objective using physical-time quadrature; budget-exit
tests exercise both implementations.

Geometric refinement policy
----------------------------

The separate shooting stage uses the following policy in MATLAB and C++:

1. Evaluate physical feasibility of the incumbent and every final restart
   candidate with the public numerical acceptance checks, independently of
   convergence status and cost-decrease policy. This includes endpoint, control
   and state constraints, Euler/RK4 diagnostics, time bounds, segment clearance
   and applicable analytic continuous-geometric checks.
2. If ``require_cost_decrease`` is true, require the candidate's physical
   objective to be no greater than the original initial objective, using the
   existing relative tolerance of 1e-10. The original initial objective is not
   replaced by a possibly infeasible projected baseline's cheaper objective.
   If false, this initial-cost condition is disabled.
3. An eligible feasible candidate can replace an ineligible incumbent regardless
   of the incumbent's smaller cost. Among eligible feasible solutions, retain
   the smallest original objective plus the separately recorded rate cost,
   with the existing 1e-8 relative score-improvement threshold.

This policy does not relax physical constraints or guarantee that the search
finds a feasible candidate. If none is eligible, the existing trajectory is
retained and the public result still undergoes its normal acceptance checks.
The MATLAB refinement record exposes baseline feasibility, baseline cost
eligibility, the original initial objective and the cost-decrease setting.
Native restart records retain physical feasibility in their last column.

Final-time support
------------------

``minimum_time`` and ``maximum_time`` are supported by the two geometric
algorithms. MATLAB uses empty values for defaults; C++ uses zero. Defaults are
``max(tf-20,5)`` and ``tf+7`` seconds. Supplied bounds and effective defaults must
be ordered. Free-time shooting still requires both bounds to be explicitly
provided. The four other public algorithms reject custom bounds, since their
legacy internal time rules do not implement those options.

Result acceptance also checks the returned physical duration against each
supplied bound. ``time_bound_violation`` reports violation in seconds and
``time_bounds_pass`` records the result. Equality at a bound is permitted;
out-of-range results cannot pass even if the solver reports convergence.
The direct normalizer/metrics check enforces supplied bounds on constructed
results too; it does not imply solver support for those bounds in other modes.

Verification and limits
-----------------------

``matlab/tests/unit/TestReturnRefinementTime.m`` adds fourteen tests. They cover
three one-iteration objective checks, overflowing final-cost rollback, separation
of histories from returned objectives, infeasible/feasible incumbents, both
cost policies, state constraints, time-bound violations and equality, unsupported
algorithms and contradictory effective defaults. Native ``test_numerics.cpp``
adds matching budget, refinement and time-bound checks to its existing group.

Fresh release verification is summarized in ``docs/validation_summary.json``.
Run ``run_all_tests`` and Release CTest to reproduce the regressions. The current
raw numerical and paired timing records are under ``experiments/``. Continuous
pendulum and thermal endpoint limitations remain outside these local checks.
