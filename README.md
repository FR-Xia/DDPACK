# DDPACK

DDPACK is a MATLAB and independent native C++17 package for selected smooth,
finite-horizon nonlinear optimal-control and structured dynamic-optimization
problems. Six package-specific DDP/PDDP implementation variants share documented
model and result contracts, deterministic examples and independent diagnostics.

**Version:** 1.1.0 | **License:** BSD-3-Clause |
[Release](https://github.com/FR-Xia/DDPACK/releases/tag/v1.1.0) |
[Issues](https://github.com/FR-Xia/DDPACK/issues)

## Install and run MATLAB

The recorded validation uses MATLAB R2023b. The formal examples require no
third-party MATLAB toolbox. From the repository root:

```matlab
addpath('matlab/tools');
setup_ddpack('IncludeExamples',true,'IncludeBenchmarks',true);
problem = ddpack.problems.linearRegulation();
solution = ddpack.solve(problem,ddpack.Options('Algorithm','free_time_pddp'));
disp(solution.status);
ddpack.plot(solution);
run_all_tests;
run_publication_examples('Visible',false,'QuickMode',false);
```

Run `setup_ddpack` rather than recursively adding generated build/output trees.
Choose new output directories for each example or benchmark run.

## Build and run C++

Requirements: a C++17 compiler, CMake >=3.20 and Eigen >=3.4. Native use requires
neither MATLAB nor MATLAB Runtime.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
./build/ddpack_examples all artifacts/native_run_001
```

Visual Studio uses executables under `build/Release/`. Configure a new Visual
Studio tree with `-A x64 -T host=x64`. An explicit Eigen source tree can be
selected with `-DDDPACK_EIGEN_DIR=/path/to/eigen`. Installation and extension
contracts are described in [cpp/README.rst](cpp/README.rst).

## Algorithms and intended scope

Maintained identifiers are `free_time_pddp`, `waypoint_pddp`,
`terminal_defect_pddp`, `geometric_safety_pddp_2d`,
`geometric_safety_pddp_3d` and `penalty_free_time_ddp`.
They are local implementation variants rather than complete reproductions of
every algorithm suggested by deprecated names. They do not directly solve
arbitrary static nonlinear programs. Custom derivatives and model signatures
must match the chosen variant. Custom final-time bounds apply only to geometric
algorithms; other modes reject them. General nonlinear mixed constraints are
outside the validated native correction contract.

Results expose convergence status, returned objective, separate iteration
histories, state/control feasibility, physical time, Euler consistency and
independent local RK4 defects. Supported geometric models additionally enforce
analytic held-control replay endpoints and conservative clearance bounds.

**Continuous-model limitation:** successful discrete acceptance and finite local
RK4 defects do not certify a complete continuous-model rollout. Nominal pendulum
and thermal continuous replays can exceed endpoint tolerances. Replay and refine
before using those trajectories where continuous endpoints are required. No
global optimality, arbitrary-initialization robustness or real-time guarantee is
claimed.

## Examples, tests and reproducibility

Nine formal examples cover vibration, pendulum, reactor, thermal regulation,
storage, planar transfer, terminal-defect transfer and planar/spatial avoidance.
The software includes MATLAB tests, CTest, frozen cross-language references,
matched derivative probes and source hashes. The verification scope and
reproduction commands are in [docs/VALIDATION.rst](docs/VALIDATION.rst).

[Paired experiments](experiments/README.rst) retain common configurations,
trajectories, physical checks and all 90 measured solves: three warmups and five
measured repeats per case/language. Timings describe the recorded machine and
implementations; they do not establish general language or solver superiority.

## Documentation and files

| Directory | Contents |
| --- | --- |
| `matlab/` | MATLAB source, examples, tests and benchmark tools |
| `cpp/` | Native library, headers, examples, tests and reference data |
| `experiments/` | Current measured evidence and analysis scripts |
| `docs/` | Validation, numerical contracts, limitations and source licenses |
| `DDPACK_Manual/` | [Chinese manual](DDPACK_Manual/DDPACK_Manual.pdf) and LaTeX source |

See [README.rst](README.rst) for the extended reference. Build trees and local
execution artifacts are excluded from the source release.

## Citation and support

Use [CITATION.cff](CITATION.cff) for software citation metadata. The corresponding
SoftwareX manuscript is in preparation; no article DOI is claimed. Contact
Feiran Xia at beihesanx@gmail.com or open a repository issue.

Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China.
[Licence.txt](Licence.txt) and [LICENSE.txt](LICENSE.txt) contain the BSD-3-Clause
license. Dependency notices are in [docs/SOURCE_LICENSES.rst](docs/SOURCE_LICENSES.rst).
