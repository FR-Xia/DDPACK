Matched derivative evidence
===========================

Nine models, three deterministic state/control probe points and three central
stencil widths produce 81 reports per language. MATLAB has 1755 checks and native
C++ 1314. Categories include center-value contracts, derivatives and symmetry;
MATLAB-only time/constraint-control contracts are not claimed as native coverage.

Raw reports retain points, options, errors and pass decisions. The source
manifest binds this release's inventoried sources and the CI workflow. Run
``python experiments/check_derivative_evidence.py`` to verify recorded evidence;
rerun both model audits after relevant source changes before packaging reports.
See ``docs/DERIVATIVE_VALIDATION.rst`` for reproduction and limitations.
