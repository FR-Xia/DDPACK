// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <ddpack/ddpack.hpp>
namespace ddpack {
bool solve_control_system(Matrix& hessian,const Matrix& rhs,Matrix& solution,NumericalDiagnostics&);
}
