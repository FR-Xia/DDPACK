Paired implementation measurements
==================================

``experiments/results`` contains the current nine-case MATLAB/native comparison.
Each implementation runs three warmups and five measured public solves per
case, sequentially on the recorded Windows machine. Setup, startup, plotting,
serialization and compilation are excluded. Public validation and diagnostics
are included. Report all repeats, medians and quartiles.

The records compare these implementations and numerical results. They do not
establish a general language speed advantage or external-solver superiority.
Reproduce using ``experiments/README.rst``. Every numerical conclusion must be
interpreted alongside its physical/discrete acceptance scope.
