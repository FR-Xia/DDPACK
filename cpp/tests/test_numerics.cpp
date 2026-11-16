// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include "numerics.hpp"
#include "problems.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
namespace ddpack {void refine_geometric(const Problem&,Solution&);}
double independentObjective(const ddpack::Problem& p,const ddpack::Solution& s) {
    double value=p.cost->terminal(s.states.col(s.states.cols()-1),p.target);
    for(Eigen::Index k=0;k<s.time.size()-1;++k)
        value+=(s.time[k+1]-s.time[k])*p.cost->running(s.states.col(k),p.target,s.controls.col(k));
    return value;
}
void checkReturnRefinementTime() {
    using namespace ddpack;
    for(const auto& name:{"planar","vibration"}) {
        auto c=examples::make(name);c.options.max_iterations=1;
        auto s=solve(c.problem,c.options);
        check(std::abs(s.returned_cost-independentObjective(c.problem,s))<1e-9,"Budget objective does not match returned trajectory");
    }
    auto c=examples::make("obstacle_2d");auto p=c.problem;
    p.cost=std::make_shared<QuadraticCost>(Matrix::Identity(3,3),Matrix::Zero(3,3),Matrix::Ones(1,1));
    const int n=static_cast<int>(std::floor(p.tf/p.dt));
    Solution baseline;baseline.states=Matrix::Zero(3,n);baseline.states.row(0)=Vector::LinSpaced(n,0,5).transpose();
    baseline.controls=Matrix::Zero(1,n);baseline.final_time=5;baseline.time=Vector::LinSpaced(n,0,5);
    baseline.initial_cost=0;baseline.returned_cost=0;
    p.require_cost_decrease=false;
    auto s=baseline;refine_geometric(p,s);
    check(s.refinement_selected_start>0,"Infeasible cheap incumbent blocked feasible candidate");
    s.time=Vector::LinSpaced(n,0,s.final_time);s.returned_cost=independentObjective(p,s);
    check(s.returned_cost>0 && compute_metrics(p,s).numerical_checks_pass,"Higher-cost feasible replacement failed acceptance");
    p.require_cost_decrease=true;s=baseline;refine_geometric(p,s);
    check(s.refinement_selected_start==0 && s.refinement_starts.col(6).maxCoeff()>0,"Required initial cost decrease was ignored");
    s=baseline;s.initial_cost=100;refine_geometric(p,s);
    check(s.refinement_selected_start>0,"Cost decrease compared against infeasible baseline instead of original initial cost");
    p.require_cost_decrease=false;p.constraints.push_back(state_lower_bound(1,.5,3));
    s=baseline;refine_geometric(p,s);
    check(s.refinement_selected_start==0 && s.refinement_starts.col(6).maxCoeff()==0,"Refinement ignored state constraints");
    p.constraints.clear();p.obstacle_centers.col(0)<<100,100;s=baseline;refine_geometric(p,s);
    check(s.refinement_selected_start==0,"Worse score replaced feasible cheap incumbent");
    for(double duration:{2.,5.,6.,7.}) {
        s=baseline;s.time=Vector::LinSpaced(n,0,duration);s.final_time=duration;
        const auto metrics=compute_metrics(p,s);
        check(metrics.time_bounds_pass==(duration>=5 && duration<=6),"Time bounds not enforced at acceptance");
        if(duration<5 || duration>6)check(!metrics.numerical_checks_pass,"Out-of-range time passed numerical checks");
    }
    for(auto algorithm:{Algorithm::free_time_pddp,Algorithm::waypoint_pddp,Algorithm::terminal_defect_pddp,Algorithm::penalty_free_time_ddp}) {
        auto bad=examples::make("planar");bad.problem.minimum_time=5;bad.problem.maximum_time=6;bad.options.algorithm=algorithm;
        bool rejected=false;try{solve(bad.problem,bad.options);}catch(const std::invalid_argument& e){
            rejected=std::string(e.what()).find("supported only by geometric")!=std::string::npos;
        }
        check(rejected,"Unsupported custom time bounds were silently accepted");
    }
    p.minimum_time=0;p.maximum_time=4;bool rejected=false;
    try{solve(p,c.options);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"Contradictory effective geometric time bounds accepted");
}
struct DiagnosticCost final:ddpack::Cost {
    ddpack::QuadraticCost q{ddpack::Matrix::Identity(2,2),ddpack::Matrix::Identity(2,2),ddpack::Matrix::Ones(1,1)};
    int mode=1;
    double terminal(const ddpack::Vector& x,const ddpack::Vector& t)const override{return q.terminal(x,t);}
    ddpack::Vector terminal_x(const ddpack::Vector& x,const ddpack::Vector& t)const override{return q.terminal_x(x,t);}
    ddpack::Matrix terminal_xx(const ddpack::Vector& x,const ddpack::Vector& t)const override{return q.terminal_xx(x,t);}
    double running(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{
        return mode==1 && u.norm()>0?ddpack::infinity:q.running(x,t,u);
    }
    ddpack::Vector running_x(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{return q.running_x(x,t,u);}
    ddpack::Vector running_u(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{return q.running_u(x,t,u);}
    ddpack::Matrix running_xx(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{return q.running_xx(x,t,u);}
    ddpack::Matrix running_uu(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{
        return mode==2?ddpack::Matrix::Constant(1,1,-1e100):q.running_uu(x,t,u);
    }
    ddpack::Matrix running_xu(const ddpack::Vector& x,const ddpack::Vector& t,const ddpack::Vector& u)const override{return q.running_xu(x,t,u);}
};
int main(){
    using namespace ddpack;
    try{
        checkReturnRefinementTime();
        Matrix h(2,2);h<<4,1,1,3;Matrix expected(2,2);expected<<1,2,-2,3;
        Matrix rhs=h*expected,answer;NumericalDiagnostics info;
        check(solve_control_system(h,rhs,answer,info),"Conditioned solve failed");
        check((answer-expected).norm()<1e-13 && info.regularized_solves==0,"Conditioned residual/damping");
        for(int kind=0;kind<3;++kind){
            h=Matrix::Zero(2,2);if(kind==1){h(0,0)=-2;h(1,1)=1;}if(kind==2){h(0,0)=1e-16;h(1,1)=1;}
            info=NumericalDiagnostics{};
            check(solve_control_system(h,Matrix::Identity(2,2),answer,info),"Damped solve failed");
            check(info.regularized_solves==1 && info.maximum_damping>0,"Missing damping diagnostics");
            check((h*answer-Matrix::Identity(2,2)).norm()<1e-10,"Damped residual");
        }
        h=Matrix::Constant(1,1,infinity);info=NumericalDiagnostics{};
        check(!solve_control_system(h,Matrix::Ones(1,1),answer,info),"Nonfinite system accepted");
        Problem p;p.x0=Vector::Zero(1);p.target=p.x0;p.u_max=Vector::Ones(1);p.tf=2;p.dt=.2;p.intervals=10;
        p.dynamics=std::make_shared<LinearDynamics>(Matrix::Zero(1,1),Matrix::Zero(1,1),Vector::Zero(1));
        p.cost=std::make_shared<QuadraticCost>(Matrix::Zero(1,1),Matrix::Zero(1,1),Matrix::Ones(1,1));
        p.require_cost_decrease=false;Options o;o.algorithm=Algorithm::penalty_free_time_ddp;o.max_iterations=2;
        auto s=solve(p,o);
        check(!s.metrics.success && s.metrics.status=="numerical_failure","Singular time reported success");
        check(s.states.allFinite() && s.controls.allFinite() && s.final_time==2,"Failed solve lost finite nominal");
        check(s.numerical_diagnostics.failure_identifier=="singular_time_hessian","Missing time failure cause");
        auto c=examples::make("planar");c.problem.require_cost_decrease=false;c.problem.terminal_tolerance=1e6;c.options.max_iterations=1;
        s=solve(c.problem,c.options);
        check(!s.metrics.success && s.metrics.status=="feasible_not_converged","Iteration cap reported convergence");
        p.x0=Vector::Zero(2);p.target=Vector::Ones(2);p.tf=10;p.dt=1;
        p.dynamics=std::make_shared<LinearDynamics>(Matrix::Zero(2,2),Matrix::Ones(2,1),Vector::Zero(2));
        auto cost=std::make_shared<DiagnosticCost>();p.cost=cost;o.algorithm=Algorithm::waypoint_pddp;o.max_iterations=3;
        s=solve(p,o);
        check(!s.metrics.success && s.metrics.status=="numerical_failure","Overflowing objective reported success");
        check(s.controls.norm()==0 && std::isfinite(s.returned_cost),"Objective overflow lost the last finite nominal");
        cost->mode=2;s=solve(p,o);
        check(!s.metrics.success && s.numerical_diagnostics.backward_retries==79,"Backward retry limit not enforced");
        std::cout<<"Conditioned/damped systems, singular-time rollback and status gates PASS\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
