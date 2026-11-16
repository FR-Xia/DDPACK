// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <ddpack/ddpack.hpp>
namespace ddpack {
struct DerivativeCheckOptions {
    double difference_step=1e-5, absolute_tolerance=1e-7, relative_tolerance=5e-6;
};
struct DerivativeCheck {
    std::string name,message,category="derivative";
    bool passed=false;
    double max_absolute_error=infinity,max_scaled_error=infinity,max_tolerance_ratio=infinity;
};
struct DerivativeReport {
    bool success=false;
    std::vector<DerivativeCheck> checks;
    DerivativeCheckOptions options;
    Vector state,control;
    std::vector<std::string> excluded_checks={"Time derivatives (native callbacks use physical-time autonomous functions)",
        "Control constraint derivatives (not exposed by the native constraint contract)",
        "Dynamics second derivatives, clipping and complete solver iterations"};
};
// Offline central-difference check at an explicit point; never calls solve().
// Dynamics use physical time; MATLAB additionally checks its time-scaling API.
DerivativeReport check_derivatives(const Problem&,const Vector& state,const Vector& control,
                                  const DerivativeCheckOptions& options={});
}
