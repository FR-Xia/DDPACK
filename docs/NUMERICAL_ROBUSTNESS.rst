Numerical safeguards and result status
======================================

The maintained solvers use bounded control-system regularization, finite
trajectory/cost checks, bounded backward retries and scalar-time-curvature
checks. Numerical failures preserve an available complete finite nominal and
retain a failure reason. Callback programming errors propagate.

``converged`` requires a reported stopping condition and configured numerical
acceptance. ``feasible_not_converged`` means numerical checks passed but an
iteration budget or other stopping status did not establish convergence.
``failed_checks`` indicates failed physical/discrete checks;
``numerical_failure`` retains the applicable numerical failure diagnostics.

These safeguards do not prove convergence or physical-model correctness.
Inspect returned states, controls, time, objective, histories and diagnostics.
Choose tolerances with meaningful units. The full continuous-model pendulum and
thermal replay limitations are documented in ``ACCEPTANCE_VALIDATION.rst``.
