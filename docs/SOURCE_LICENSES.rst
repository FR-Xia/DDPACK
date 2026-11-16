Source and dependency licenses
=============================

DDPACK project sources
----------------------

Copyright (c) 2026 Feiran Xia, Beijing Institute of Technology, China.
DDPACK project sources are distributed under BSD-3-Clause; the full license is
provided in ``LICENSE.txt`` and ``Licence.txt``. The same project license applies
to the MATLAB and native C++ implementations, examples, tests and support tools.

``matlab/tools/audit_source_licenses.m`` generates a per-file inventory with paths,
module roles, declared license, copyright holder and SHA-256. Its published
inventory is in ``docs/source_licenses/``. It checks project notices against the
declared license; it is not an inventory of all packages installed on a user's
computer.

Module relationships
--------------------

* ``matlab/src/algorithms/`` contains six maintained DDP/PDDP variants and
  compatibility entry points. The registry describes their validated scope.
* ``cpp/src/solver.cpp`` implements the corresponding MATLAB algorithm branches
  natively in C++17; it does not call MATLAB or MEX.
* ``cpp/src/models.cpp`` implements the native dynamics and cost models.
* MATLAB and C++ diagnostics and geometric refinement are separate project modules.
* Examples, tests and analysis scripts are included under the project license.
* Cited research papers describe theoretical context; their authors and references
  remain bibliographic credits, not a declaration that their article licenses
  license DDPACK source code.

External dependencies
---------------------

These dependencies are installed separately. Their source trees and license
notices are not replaced by DDPACK's copyright or BSD license.

* **Eigen** (native linear algebra; >=3.4; recorded local version 5.0.0): primarily
  MPL-2.0, with additional notices for included third-party files. See the exact
  installed version's ``COPYING.README`` and ``COPYING.*``. DDPACK does not vendor
  Eigen. Official licensing information:
  https://libeigen.gitlab.io/pages/licensing_faq/
* **MATLAB** (MATLAB implementation and its tests; verified environment R2023b):
  separately licensed MathWorks software, not bundled or relicensed by DDPACK.
  The native C++ implementation does not require MATLAB or MATLAB Runtime.
* **NumPy** (experiment analysis; requirements in ``experiments/requirements.txt``):
  BSD-3-Clause. Official license: https://numpy.org/doc/stable/license.html
* **Matplotlib** (experiment plots): its own PSF-based license and bundled
  component notices. Official license: https://matplotlib.org/stable/project/license.html
* **CMake, CTest, Git, C++ compilers and TeX tools** are separately installed
  build/development tools governed by their own distribution licenses.
* **GitHub Actions** referenced by workflow files are externally maintained CI
  tools; they are not part of the DDPACK source license.

Reproduction and packaging
--------------------------

Run ``addpath('matlab/tools'); setup_ddpack; audit_source_licenses`` from the
repository root. Use ``'OutputDir'`` to export a refreshed inventory. Release
packaging includes this document and the generated inventory through ``docs/``.
When redistributing dependency files or binaries, retain the notices required by
the exact dependency distributions used.

Recorded experiment provenance
-------------------------------

The 2026-10-22 measurements remain unchanged. Original source-byte hashes are
preserved as ``experiments/results/source_sha256_20261022.csv`` and
``cpp/tests/reference/MATLAB_SOURCE_SHA256_20261022.csv``. Current-source manifests
are linked to them by ``experiments/results/source_notice_changes.csv``.
The link distinguishes copyright-comment changes from license/documentation
tooling changes; numerical source bodies were compared with the frozen baseline.
This administrative update does not represent new timing measurements.
Subsequent command-line Unicode handling and build/test changes are linked by
``experiments/results/source_cli_changes.csv`` to
``experiments/results/source_sha256_pre_unicode_20261113.csv``. The numerical
library and MATLAB sources remain unchanged; original measurements are retained.
