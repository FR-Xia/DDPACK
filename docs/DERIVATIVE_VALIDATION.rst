Offline derivative validation
=============================

``ddpack.checkDerivatives`` and native ``ddpack::check_derivatives`` compare
analytic dynamics, cost and applicable constraint derivatives against
component-scaled central differences. Hessians use differences of first
gradients; symmetry and center-point value contracts are checked separately.
Defaults are absolute tolerance 1e-7 and relative tolerance 5e-6.

The nine formal models use three deterministic points and three relative
stencil widths (1e-4, 1e-5, 1e-6): 81 reports in each language, comprising 1755
MATLAB and 1314 native value/derivative/symmetry checks. MATLAB additionally
checks normalized-time and constraint-control contracts absent from the native
autonomous interface. Results are in ``experiments/derivative_results``.

From MATLAB, choose a fresh output directory::

   addpath('matlab/tools');
   run_derivative_audit(fullfile(pwd,'artifacts','my_derivatives','matlab'));
   audit_source_licenses('OutputDir','docs/source_licenses');

Then run the native executable and package/verify the records::

   ./build/Release/ddpack_derivative_tests artifacts/my_derivatives/cpp
   python experiments/check_derivative_evidence.py --matlab artifacts/my_derivatives/matlab --cpp artifacts/my_derivatives/cpp --output artifacts/my_derivatives/packaged
   python experiments/check_derivative_evidence.py

The verifier uses the standard Python library. These local checks do not prove
global derivative correctness, correct physical modeling or optimizer convergence.
