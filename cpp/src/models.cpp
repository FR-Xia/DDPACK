// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/ddpack.hpp>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ddpack {
LinearDynamics::LinearDynamics(Matrix a, Matrix b, Vector d)
    : A(std::move(a)), B(std::move(b)), drift(std::move(d)) {
    if (A.rows() == 0 || A.rows() != A.cols() || B.rows() != A.rows() ||
        B.cols() == 0 || drift.size() != A.rows() || !A.allFinite() ||
        !B.allFinite() || !drift.allFinite()) throw std::invalid_argument("Invalid linear model");
}
Vector LinearDynamics::f(const Vector& x, const Vector& u) const { return A*x+B*u+drift; }
Matrix LinearDynamics::fx(const Vector&, const Vector&) const { return A; }
Matrix LinearDynamics::fu(const Vector&, const Vector&) const { return B; }
Vector PendulumDynamics::f(const Vector& x, const Vector& u) const {
    Vector v(2); v << x[1], (mass*gravity*length*std::sin(x[0])+u[0]-damping*x[1])/(mass*length*length); return v;
}
Matrix PendulumDynamics::fx(const Vector& x, const Vector&) const {
    Matrix a(2,2); a << 0,1,gravity/length*std::cos(x[0]),-damping/(mass*length*length); return a;
}
Matrix PendulumDynamics::fu(const Vector&, const Vector&) const {
    Matrix b(2,1); b << 0,1/(mass*length*length); return b;
}
Vector PlanarDynamics::f(const Vector& x, const Vector& u) const {
    Vector v(3); v << speed*std::cos(x[2]),speed*std::sin(x[2]),u[0]; return v;
}
Matrix PlanarDynamics::fx(const Vector& x, const Vector&) const {
    Matrix a=Matrix::Zero(3,3); a(0,2)=-speed*std::sin(x[2]); a(1,2)=speed*std::cos(x[2]); return a;
}
Matrix PlanarDynamics::fu(const Vector&, const Vector&) const {
    Matrix b=Matrix::Zero(3,1); b(2,0)=1; return b;
}
Vector FlightDynamics::f(const Vector& x, const Vector& u) const {
    Vector v(6); v << x[3]*std::cos(x[4])*std::cos(x[5]),x[3]*std::cos(x[4])*std::sin(x[5]),x[3]*std::sin(x[4]),0,u[1],u[0]; return v;
}
Matrix FlightDynamics::fx(const Vector& x, const Vector&) const {
    Matrix a=Matrix::Zero(6,6);
    const double s=x[3],g=x[4],h=x[5];
    a(0,3)=std::cos(g)*std::cos(h); a(0,4)=-s*std::sin(g)*std::cos(h); a(0,5)=-s*std::cos(g)*std::sin(h);
    a(1,3)=std::cos(g)*std::sin(h); a(1,4)=-s*std::sin(g)*std::sin(h); a(1,5)=s*std::cos(g)*std::cos(h);
    a(2,3)=std::sin(g); a(2,4)=s*std::cos(g); return a;
}
Matrix FlightDynamics::fu(const Vector&, const Vector&) const {
    Matrix b=Matrix::Zero(6,2); b(5,0)=1; b(4,1)=1; return b;
}
Vector ThermalDynamics::f(const Vector& x, const Vector& u) const {
    Vector v(1); v << -.25*x[0]-.03*std::pow(x[0],3)+.6*u[0]; return v;
}
Matrix ThermalDynamics::fx(const Vector& x, const Vector&) const { return Matrix::Constant(1,1,-.25-.09*x[0]*x[0]); }
Matrix ThermalDynamics::fu(const Vector&, const Vector&) const { return Matrix::Constant(1,1,.6); }
QuadraticCost::QuadraticCost(Matrix qf, Matrix q, Matrix r)
    : Qf(std::move(qf)), Q(std::move(q)), R(std::move(r)) {
    if (Qf.rows()==0 || Qf.rows()!=Qf.cols() || Q.rows()!=Qf.rows() || Q.cols()!=Qf.cols() ||
        R.rows()==0 || R.rows()!=R.cols() || !Qf.allFinite() || !Q.allFinite() || !R.allFinite() ||
        !Qf.isApprox(Qf.transpose()) || !Q.isApprox(Q.transpose()) || !R.isApprox(R.transpose()))
        throw std::invalid_argument("Invalid quadratic weights");
}
double QuadraticCost::terminal(const Vector& x, const Vector& t) const { Vector d=x-t; return .5*d.dot(Qf*d); }
Vector QuadraticCost::terminal_x(const Vector& x, const Vector& t) const { return Qf*(x-t); }
Matrix QuadraticCost::terminal_xx(const Vector&, const Vector&) const { return Qf; }
double QuadraticCost::running(const Vector& x, const Vector& t, const Vector& u) const { Vector d=x-t; return .5*(d.dot(Q*d)+u.dot(R*u)); }
Vector QuadraticCost::running_x(const Vector& x, const Vector& t, const Vector&) const { return Q*(x-t); }
Vector QuadraticCost::running_u(const Vector&, const Vector&, const Vector& u) const { return R*u; }
Matrix QuadraticCost::running_xx(const Vector&, const Vector&, const Vector&) const { return Q; }
Matrix QuadraticCost::running_uu(const Vector&, const Vector&, const Vector&) const { return R; }
Matrix QuadraticCost::running_xu(const Vector& x, const Vector&, const Vector& u) const { return Matrix::Zero(x.size(),u.size()); }
Constraint state_lower_bound(int index, double lower, int dimension) {
    if(index<0 || index>=dimension || !std::isfinite(lower)) throw std::invalid_argument("Invalid state bound");
    return {[index,lower](const Vector& x,const Vector&){return x[index]-lower;},
        [index,dimension](const Vector&,const Vector&){Vector v=Vector::Zero(dimension); v[index]=1; return v;}};
}
std::string algorithm_name(Algorithm a) {
    switch(a) {
        case Algorithm::free_time_pddp: return "free_time_pddp";
        case Algorithm::waypoint_pddp: return "waypoint_pddp";
        case Algorithm::terminal_defect_pddp: return "terminal_defect_pddp";
        case Algorithm::geometric_safety_pddp_2d: return "geometric_safety_pddp_2d";
        case Algorithm::geometric_safety_pddp_3d: return "geometric_safety_pddp_3d";
        case Algorithm::penalty_free_time_ddp: return "penalty_free_time_ddp";
    }
    throw std::invalid_argument("Unknown algorithm");
}
} // namespace ddpack
