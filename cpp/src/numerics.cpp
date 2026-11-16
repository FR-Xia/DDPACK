// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include "numerics.hpp"
#include <algorithm>
#include <cmath>
namespace ddpack {
bool solve_control_system(Matrix& hessian,const Matrix& rhs,Matrix& solution,NumericalDiagnostics& diagnostics) {
    if(!hessian.allFinite() || !rhs.allFinite() || hessian.rows()==0 ||
       hessian.rows()!=hessian.cols() || rhs.rows()!=hessian.rows()) {
        diagnostics.failure_identifier="nonfinite_or_invalid_control_system";return false;
    }
    const Matrix base=(hessian+hessian.transpose()).eval()/2;
    const double scale=std::max(1.,base.cwiseAbs().rowwise().sum().maxCoeff());
    if(!base.allFinite() || !std::isfinite(scale)) {
        diagnostics.failure_identifier="nonfinite_control_system_scale";return false;
    }
    for(int attempt=0;attempt<12;++attempt) {
        const double damping=attempt?scale*std::pow(10.,attempt-9):0;
        Matrix candidate=base+damping*Matrix::Identity(base.rows(),base.cols());
        if(!candidate.allFinite())continue;
        Eigen::LLT<Matrix> chol(candidate);Eigen::FullPivLU<Matrix> lu(candidate);
        if(chol.info()!=Eigen::Success || lu.rcond()<1e-12)continue;
        solution=lu.solve(rhs);
        const double residual=(candidate*solution-rhs).cwiseAbs().rowwise().sum().maxCoeff();
        const double normH=candidate.cwiseAbs().rowwise().sum().maxCoeff();
        const double normX=solution.cwiseAbs().rowwise().sum().maxCoeff();
        const double normB=rhs.cwiseAbs().rowwise().sum().maxCoeff();
        if(solution.allFinite() && residual<=1e-10*std::max(1.,normH*normX+normB)) {
            hessian=candidate;++diagnostics.linear_solves;
            diagnostics.regularized_solves+=damping>0?1:0;
            diagnostics.maximum_damping=std::max(diagnostics.maximum_damping,damping);return true;
        }
    }
    diagnostics.failure_identifier="regularization_exhausted";return false;
}
}
