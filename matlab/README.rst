DDPACK MATLAB package
=====================

This directory retains the complete MATLAB implementation, including historical
and experimental scripts. ``src/`` contains the library and public ``ddpack`` API;
``examples/`` contains the formal and historical scenarios; ``tests/`` contains
unit, regression and integration tests; ``benchmarks/`` contains benchmarks and
the paired-language runner; ``tools/`` contains setup and validation tools.

Start from the repository root (the parent of this directory)::

   addpath('matlab/tools');
   setup_ddpack('IncludeExamples',true,'IncludeBenchmarks',true);
   run_all_tests;
   run_publication_examples('Visible',false,'QuickMode',false);

Do not recursively add the entire repository: generated historical release copies
can shadow the active package. Setup adds only this MATLAB tree. Shared documents
and generated outputs live outside this directory. The supported formal scope is
the six maintained variants and nine documented cases; retention of older scripts
does not expand that scope. See the root README.md and DDPACK_Manual/.

Analytic geometric validation
-----------------------------

Supported constant-speed examples expose ``ddpack.verifyGeometricRollout(problem, controls, time)``.
It replays held controls analytically from x0 and computes a speed-based clearance
lower bound between samples. The lower bound and replay endpoint are included
in success checks; unsupported dynamics report this check as not applicable.
See ``docs/ACCEPTANCE_VALIDATION.rst`` for the numerical cushion and assumptions.
