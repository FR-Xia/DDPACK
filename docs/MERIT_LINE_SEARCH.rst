Actual-merit backtracking
=========================

Terminal-defect and geometric primary stages evaluate actual bounded rollouts.
The merit combines original objective, endpoint defect and, for geometric modes,
penetration into solver-inflated obstacles. At most 24 fractions are tried with
a fixed merit weight within an iteration. Acceptance requires actual reduction;
complete rejection retains the preceding state/control/time tuple.

Trial records expose fractions, objective, defect, time, weights, finite checks
and decisions. Shooting and terminal projection are separate explicit phases.
These are package-specific numerical rules, not a global convergence or formal
control-barrier-function invariance proof. Tests exercise acceptance, rejection,
rollback and cross-language trial records. See ``VALIDATION.rst``.
