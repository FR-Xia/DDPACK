MATLAB / native paired evidence
===============================

The nine formal cases share grids, costs, bounds, initialization and iteration
budgets. Vibration uses explicitly exported MATLAB initial controls for native
parity. MATLAB restores the RNG before each solve. All final state/control
trajectories use the documented normalized 2e-6 comparison tolerance; returned
objectives are independently recomputed with physical-time quadrature.

Three warmups and five measured public solves are run per case/language,
sequentially on one machine. Public validation and diagnostics are timed;
startup, setup, file output, plots and compilation are excluded. All repeats,
medians and quartiles are retained. These descriptive measurements do not
establish a general language or external-solver advantage.

From MATLAB::

   addpath('matlab/tools');
   setup_ddpack('IncludeBenchmarks',true);
   run_language_comparison(fullfile(pwd,'artifacts','my_comparison','matlab'));

After MATLAB finishes, run native measurement and analysis::

   ./build/Release/ddpack_benchmark cpp/tests/reference artifacts/my_comparison/cpp
   python experiments/analyze.py --matlab artifacts/my_comparison/matlab --cpp artifacts/my_comparison/cpp --output experiments/results --measurement-date YYYY-MM-DD

Analysis dependencies are in requirements.txt. ``results/raw`` retains both
languages' trajectories, configurations, metrics and timing files. It can be
reanalyzed offline. Regenerating timings requires rerunning the solvers and
produces machine-dependent values. ``source_sha256.csv`` binds the generating
sources. The derivative records have their own protocol and source inventory.
