#include <ddpack/ddpack.hpp>
#include <ddpack/derivatives.hpp>
int main() {
    using namespace ddpack;
    Problem p; p.x0=Vector::Constant(1,4); p.target=Vector::Zero(1); p.u_max=Vector::Ones(1);
    p.dynamics=std::make_shared<LinearDynamics>(Matrix::Constant(1,1,-.3),Matrix::Constant(1,1,.02),Vector::Zero(1));
    p.cost=std::make_shared<QuadraticCost>(Matrix::Constant(1,1,200),Matrix::Constant(1,1,2),Matrix::Constant(1,1,2));
    p.terminal_tolerance=.04; Options o; o.algorithm=Algorithm::penalty_free_time_ddp; o.alpha=.25;
    if(!check_derivatives(p,p.x0,Vector::Constant(1,.13)).success)return 2;
    return solve(p,o).metrics.success?0:1;
}
