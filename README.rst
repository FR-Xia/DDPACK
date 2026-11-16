DDPACK 1.1.0
==============

DDPACK supplies six package-specific implementation variants for selected
finite-horizon nonlinear optimal-control and dynamic-optimization problems.
A successful recorded case does not imply global optimality, arbitrary-problem
convergence or continuous-model endpoint feasibility.

MATLAB quick start
------------------

From the package root in MATLAB R2023b::

   addpath('matlab/tools');
   setup_ddpack('IncludeExamples',true,'IncludeBenchmarks',true);
   problem = ddpack.problems.linearRegulation();
   solution = ddpack.solve(problem,ddpack.Options('Algorithm','free_time_pddp'));
   disp(solution.status);
   run_all_tests;
   run_publication_examples('Visible',false,'QuickMode',false);
   run_benchmarks;

The formal examples require no additional MATLAB toolbox. Avoid recursively
adding generated copies of the package. Output folders must be new.

Native C++
----------

Build with CMake >=3.20, a C++17 compiler and Eigen >=3.4. MATLAB and MATLAB
Runtime are unnecessary for native use. See ``cpp/README.rst`` for installation,
an installed-library consumer and callback contracts.

Implementation scope and differences
------------------------------------

These are package-specific implementation variants:
``free_time_pddp``, ``waypoint_pddp``, ``terminal_defect_pddp``,
``geometric_safety_pddp_2d``, ``geometric_safety_pddp_3d`` and
``penalty_free_time_ddp``. The algorithm registry records derivative signatures,
theory relationships, deprecated aliases and limitations. Native and MATLAB
extensions have documented differences. General parameter vectors, general
multiple-shooting state decisions, nonsmooth/hybrid/integer problems and formal
continuous-time safety guarantees are outside the validated scope.

Validation and interpretation
-----------------------------

``objective`` describes the actual returned trajectory, independently of the
iteration histories. All-node constraints and complete dynamics diagnostics
participate in acceptance. A finite feasible result is not automatically a
convergence report. The pendulum and thermal examples demonstrate discrete
solutions; their full continuous-model replay endpoints may exceed tolerances.

The Chinese technical/user manual is ``DDPACK_Manual/DDPACK_Manual.pdf``.
``docs/VALIDATION.rst`` and ``experiments/README.rst`` describe tests, current raw
data and source bindings. Use ``ddpack.checkDerivatives`` or the native derivative
checker to audit a new model before solving it. ``docs/NUMERICAL_ROBUSTNESS.rst``
and ``docs/RETURN_REFINEMENT_TIME.rst`` explain statuses and selection rules.

The code license is BSD-3-Clause. A fixed-version link is
https://github.com/FR-Xia/DDPACK/releases/tag/v1.1.0.
Support: https://github.com/FR-Xia/DDPACK/issues or beihesanx@gmail.com.
