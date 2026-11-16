#pragma once
// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/ddpack.hpp>

namespace ddpack {
struct DefectLineSearchResult {
    bool accepted=false;
    Matrix states,controls;
    double time=0,cost=0,merit=0;
    std::vector<std::vector<double>> trials;
};
DefectLineSearchResult defect_line_search(const Problem&,const Options&,const Matrix&,const Matrix&,
    double,double,double,double,const std::vector<Vector>&,const std::vector<Matrix>&,
    const std::vector<Vector>&,double,int);
}
