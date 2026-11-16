Release verification
====================

Machine-readable release results are in ``validation_summary.json``. Formal
scope comprises six maintained variants and nine deterministic examples.

In MATLAB R2023b from the package root::

   addpath('matlab/tools');
   setup_ddpack('IncludeExamples',true,'IncludeBenchmarks',true);
   run_all_tests;
   run_publication_examples('OutputDir',fullfile(pwd,'artifacts','my_examples'),'Visible',false,'QuickMode',false);

For the native library, configure/build with CMake and run CTest as described
in the root README. CTest includes physical/discrete acceptance, fresh-reference
parity, source integrity, actual-merit decisions, numerical safeguards,
derivative counterexamples and Unicode paths. Build an installed-library
consumer with ``cpp/tests/consumer`` to check installation.

Both derivative audits cover 81 matched probes per language. The Python evidence
verifier checks every record and source binding. Paired raw records, parameters,
timings and source inventories are in ``experiments/results``. Follow
``experiments/README.rst`` to rerun or regenerate tables.

Passing regression tests do not establish arbitrary-grid or initialization
robustness, global convergence, continuous endpoints for every model, or safety
certification. Full continuous pendulum/thermal endpoints can exceed tolerances.
