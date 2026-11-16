DDPACK native C++ library
========================

This directory adds an independent C++17 implementation alongside the complete
MATLAB package. MATLAB, MATLAB Runtime, MEX, and MATLAB Coder are not required
to build or run it. MATLAB entry points, examples and tests remain available under ``matlab/``.
Start MATLAB from the repository root with ``addpath('matlab/tools')``. Both
implementations are part of the 1.1.0 dual-language release candidate.

Build and run
-------------

Requirements: CMake >= 3.20, a C++17 compiler and Eigen >= 3.4. Install Eigen
with your system package manager (for example, ``libeigen3-dev`` on Ubuntu).
An existing Eigen source tree can be supplied with
``-DDDPACK_EIGEN_DIR=/path/to/eigen``. Dependencies are not downloaded by CMake.

From the repository root::

   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build --config Release --parallel 2
   ctest --test-dir build -C Release --output-on-failure
   ./build/ddpack_examples all artifacts/native_run_001

For Visual Studio, use ``-G "Visual Studio 17 2022" -A x64`` when configuring.
Executables are then under ``build/Release/``. Select a new output directory
on each run. The runner refuses to overwrite existing evidence. A single
example can be selected, e.g. ``ddpack_examples obstacle_3d output/new_run``.

Unicode command-line paths
--------------------------

The example runner, benchmark runner and parity checker accept Unicode input and
output paths, including absolute paths and spaces. On Windows they receive UTF-16
arguments through ``wmain`` and explicitly convert them to UTF-8 before calling
``std::filesystem::u8path``. Other platforms retain their UTF-8 argument handling.
The library's ``write_artifacts`` string path is also UTF-8.

For example, from the repository root in PowerShell::

   $destination = Join-Path (Get-Location) 'artifacts/中文结果 001'
   & .\build\Release\ddpack_examples.exe all $destination

Choose a destination that does not already exist; paths with spaces must be
passed as one argument. No system code-page or locale change is required.
``ctest`` includes argument-conversion and end-to-end Unicode-path regression
checks. The latter runs all nine examples, parity checks and benchmark writes,
and verifies that an existing output directory cannot be overwritten.

Library use
-----------

Custom ``minimum_time``/``maximum_time`` bounds apply only to the two geometric
algorithms; zero selects the algorithm default. Other algorithms reject these
options instead of ignoring them. The returned duration is checked at acceptance
and reported through ``time_bounds_pass`` and ``time_bound_violation`` (seconds).
Geometric refinement prioritizes eligible physical feasibility over an infeasible
incumbent's lower cost. ``require_cost_decrease`` compares against the original
initial objective; eligible feasible candidates are ranked by objective plus
rate cost. See ``docs/RETURN_REFINEMENT_TIME.rst`` for the full policy and tests.

Link against ``DDPACK::ddpack``. The example factory is a separate development
target, not part of the installed public library. Minimal custom problem::

   #include <ddpack/ddpack.hpp>
   using namespace ddpack;
   Problem p;
   p.x0 = Vector::Constant(1, 4);
   p.target = Vector::Zero(1);
   p.u_max = Vector::Ones(1);
   p.dynamics = std::make_shared<LinearDynamics>(
       Matrix::Constant(1,1,-0.3), Matrix::Constant(1,1,0.02), Vector::Zero(1));
   p.cost = std::make_shared<QuadraticCost>(
       Matrix::Constant(1,1,200), Matrix::Constant(1,1,2), Matrix::Constant(1,1,2));
   Options options;
   options.algorithm = Algorithm::penalty_free_time_ddp;
   options.alpha = 0.25;
   p.terminal_tolerance = 0.04;
   p.rk4_tolerance = 0.02;
   Solution solution = solve(p, options);

Install with ``cmake --install build --config Release --prefix <destination>``.
Consumers use ``find_package(DDPACK CONFIG REQUIRED)`` and
``target_link_libraries(my_app PRIVATE DDPACK::ddpack)``. Installed consumers
need an Eigen CMake package or the same explicit ``DDPACK_EIGEN_DIR`` option.

Interfaces and numerical conventions
-----------------------------------

* All states and controls are column vectors; trajectories store one node in
  each column. Controls have N columns, with the final sample held from N-1.
  There are N-1 physical integration intervals.
* Dynamics callbacks provide physical-time ``f``, ``fx`` and ``fu``. They must
  return finite values with consistent dimensions. Do not apply final-time
  scaling inside the callbacks; the solver applies it.
* Costs provide an autonomous running-cost density and autonomous terminal
  cost with analytic first and second derivatives. State-control mixed
  derivatives are supported. Explicit time-dependent cost functions and
  dynamics are outside the native callback contract.
* Scalar state constraints use ``g(x,u) >= 0`` and ``gx``. The solver multiplies
  both the value and gradient by the physical time step for the backward
  correction, matching MATLAB's step-weighted constraint interface. Callbacks
  must return unweighted values. As in the current
  MATLAB constraint correction, only state gradients enter the backward
  penalty. Use ``u_max`` for control bounds. General mixed nonlinear control
  constraints are outside this correction's scope. Constraint callbacks are
  accepted only by free-time PDDP and penalty free-time DDP; the safety branches
  use their obstacle geometry fields.
* ``intervals`` is the number of physical intervals for
  ``penalty_free_time_ddp`` (N = intervals + 1). Other algorithms retain the
  legacy N = floor(tf/dt) convention, then use a normalized Euler grid.
* Native input validation rejects invalid grids/options and singular
  guidance at an obstacle center or target. Guidance-based algorithms retain
  the legacy fixed 0.1-second seed: dt must be >= 0.1. Terminal-defect guidance
  requires three states and one control; geometric variants require the
  documented planar (3/1) or flight (6/2) state/control layouts.
* ``initial_controls`` allows explicit deterministic initialization for
  free-time and waypoint solves. Without it the free-time solver uses a fixed
  C++ seed; C++ and MATLAB random generators do not generate identical
  sequences. Parity uses the exact MATLAB initial controls from a fixture.
* The two formal waypoint examples have no intermediate waypoint. The native
  segmented waypoint path requires equal integer grids; broader segmented
  and free-segment-time behavior needs separate application validation.

Algorithm mapping
-----------------

All six public MATLAB identifiers have separate branches in the native solver:

* ``free_time_pddp``: coupled state/control/time value recursion and bounded
  Newton update of the final time.
* ``waypoint_pddp``: regularized state/control/time recursion, optional
  segment-time updates and intermediate position penalties.
* ``terminal_defect_pddp``: terminal-defect terms in the last backward step
  and bounded actual-merit backtracking.
* ``geometric_safety_pddp_2d`` and ``geometric_safety_pddp_3d``: terminal-defect
  recursion plus the source geometric control correction and optional
  feasibility-preserving terminal projection.
* ``penalty_free_time_ddp``: projected terminal-time value derivatives,
  state-constraint penalty corrections and strict best nominal iterate return.

These remain package-specific variants with the limitations documented by
``matlab/src/+ddpack/algorithmRegistry.m``. They are local methods. Geometric
corrections do not establish continuous-time safety or CBF invariance.
The native library does not port experimental/legacy scripts, MATLAB plotting,
MAT serialization, or deprecated aliases. Those remain in the complete MATLAB
distribution.

The port uses linear solves instead of explicit matrix inverses, bounded
regularization retries and explicit numerical failure statuses. The terminal
projection uses the algebraically equivalent dual damped least-squares system.
Terminal-defect and geometric branches use bounded actual-merit backtracking
with transactional candidate rollouts and rollback on rejection. Merit combines
cost, terminal error and, for geometric variants, exact segment penetration.
Guidance controls are first re-integrated on the solver grid. See
``docs/MERIT_LINE_SEARCH.rst`` for weights, trial schema and limitations.

Evidence and diagnostics
------------------------

``<ddpack/derivatives.hpp>`` exposes an offline ``check_derivatives`` model audit.
It checks physical-time dynamics, cost gradients/Hessians/mixed directions and
native constraint state gradients against central differences, with separate
finite-value/dimension checks. ``ddpack_derivative_tests [fresh_output_dir]`` runs
81 formal probes and intentional counterexamples. See
``docs/DERIVATIVE_VALIDATION.rst`` and ``experiments/derivative_results`` for
checked fields, excluded callbacks and raw records. This API is not called by
the optimizer and is not an automatic-differentiation or convergence guarantee.

All six maintained branches use finite, conditioned control solves with at most
12 diagonal-damping attempts. Waypoint backward retries are capped at 80 and
scalar time curvature is checked before division. A numerical failure retains
the finite nominal and is exported with its cause. Iteration caps and stalled
projection are not convergence reports even if physical checks pass. JSON metrics
separate ``numerical_checks_pass``, ``solver_convergence_pass`` and ``status``.
See ``docs/NUMERICAL_ROBUSTNESS.rst`` for equations, diagnostics and boundaries.

``ddpack_tests`` checks Jacobians by central differences, deterministic solves,
input rejection, segment collision detection and all nine native examples.
``ddpack_parity`` compares final states, controls, time and phase-specific histories
with MATLAB R2023b fixtures. All first-update trajectories/active controls use
a relative-to-maximum-scale tolerance of 1e-9. All nine final trajectories now use the same 2e-6 state/control tolerance;
canonical geometric restart records and merit histories use 1e-8 and returned
objectives use 1e-9. Primary line-search trial matrices and actual-merit histories of the three
affected variants are also checked at 1e-8. Broader historical 3D tolerances have been removed
from final-trajectory checks. See the trajectory-quality stage below.
Non-geometric trial histories use 2e-5. The comparison scale is max(1, max(abs(reference))).
The original physical acceptance thresholds are tested independently for
every full run; these are not relaxed to accommodate language differences.
The unused terminal control is excluded from parity because the native API
always holds the final active control while MATLAB can retain a guidance seed.
For geometric cases the MATLAB cost history also contains the actual
post-projection objective, recorded separately from primary merit. Tests cover numerical agreement within
declared tolerances, not bitwise equality. Frozen references allow native CI
without MATLAB; a separate CI job regenerates them using MATLAB.

Regenerate into a fresh directory in MATLAB::

   addpath('matlab/tools');
   export_cpp_reference(fullfile('artifacts', 'new_reference'));

Then run ``ddpack_parity artifacts/new_reference``. Review changes before
replacing frozen references. Reference generation evaluates only the setup
prefix of the repository's trusted formal example files, up to the solver call;
it does not execute downloaded input or user-provided configuration.

The runner writes states, controls, physical time, trial/best cost histories,
JSON metrics, configuration and compiler/Eigen information. Non-applicable
clearance is JSON ``null``. The authoritative problem definitions are in
``cpp/examples/problems.cpp``. RK4 uses four substeps per held-control interval;
Euler and RK4 defects are maximum absolute component errors, matching MATLAB.
Acceptance checks terminal error, bounds, constraints, segment clearance,
initial state, cost decrease and the configured defect tolerances. A numerical
failure is reported with its cause, and iteration exhaustion is not
accepted as convergence. Feasibility alone does not imply stationarity. Numerical failure terminations
(nonfinite trajectory, singular Hessian or failed backward pass) fail native
acceptance.

``ddpack_benchmark cpp/tests/reference artifacts/native_benchmark_001`` runs
three warmups and five timed repetitions per case, uses matched MATLAB initial
controls for vibration, and records each run's acceptance. Timings include
``solve`` and its numerical diagnostics, excluding file output. Record build
configuration, compiler, Eigen version, CPU/OS and both code revisions before
interpreting comparisons; a single local run does not establish a general
performance improvement.

Source attribution
------------------

Native recursions and models are the C++ implementation of the MATLAB algorithms and models in this repository. Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China. Project sources are distributed under BSD-3-Clause; see LICENSE.txt. Eigen is an external dependency with its own license and is not vendored here. See docs/SOURCE_LICENSES.rst for module mapping and dependency notices.

Geometric trajectory-quality stage (2026-10-21)
-----------------------------------------------

The formal 2D/3D cases now opt into bounded multistart shooting refinement via
``geometric_refinement_iterations=80``, ``geometric_refinement_knots=12``,
``geometric_control_rate_weight=.02`` and ``geometric_refinement_free_time=true``.
Library defaults keep refinement disabled. The independent canonical cosine
restarts optimize control knots and time, then reintegrate dynamics. This is an
explicit additional numerical stage, not a claim that the historical PDDP/CBF
heuristic is a standard constrained optimizer or that its result is globally
optimal. It currently requires QuadraticCost. Formal time bounds are [5,6] s;
old [5.2,5.25] s results are retained in the ablation. Acceptance preserves the
original physical obstacle radius, safety margin, control and terminal bounds.
Reported returned_cost is the original quadratic cost; the rate regularizer and
per-start records are separate artifacts. An infeasible or unimproved candidate
cannot replace the pre-refinement result.

Final state/control parity now uses 2e-6 for all nine cases, including 3D; restart
records/merit use 1e-8 and final objectives 1e-9. Earlier broad 3D final-trajectory
tolerances described above apply only to the archived pre-refinement results.
Heuristic primary traces may branch under rounding and remain diagnostic output;
canonical restart output is tested independently. See docs/ACCEPTANCE_VALIDATION.rst.

Analytic held-control acceptance
-------------------------------

For PlanarDynamics and FlightDynamics with static circle/sphere obstacles,
diagnostics independently integrate each held control in closed form from x0.
At 64 subintervals per control interval, the minimum sampled clearance minus
speed times half the maximum subinterval is a lower bound between samples.
A 1e-10*(1+norm(x0)+speed*T) numerical cushion is additionally subtracted.
``Metrics::analytic_zoh_clearance_lower_bound`` must be nonnegative and
``analytic_zoh_terminal_error`` must meet terminal_tolerance for success.
Other models report NaN for these fields. This floating-point, model-specific
check is not interval arithmetic, disturbance robustness, or global optimality.
MATLAB exposes the corresponding audit through ddpack.verifyGeometricRollout.
