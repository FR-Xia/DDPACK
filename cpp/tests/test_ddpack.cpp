#include "problems.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool c,const char* message) {if(!c) throw std::runtime_error(message);}
int main() {
    try {
        // Analytic derivatives against independent central differences.
        for(const auto& name:ddpack::examples::names()) {
            auto c=ddpack::examples::make(name); const auto& d=*c.problem.dynamics;
            ddpack::Vector x=c.problem.x0,u=ddpack::Vector::Constant(c.problem.u_max.size(),.13);
            ddpack::Matrix a=d.fx(x,u),b=d.fu(x,u);
            for(int j=0;j<x.size();++j) {auto xp=x,xm=x; xp[j]+=1e-6; xm[j]-=1e-6; check(((d.f(xp,u)-d.f(xm,u))/2e-6-a.col(j)).norm()<1e-7,"State Jacobian");}
            for(int j=0;j<u.size();++j) {auto up=u,um=u; up[j]+=1e-6; um[j]-=1e-6; check(((d.f(x,up)-d.f(x,um))/2e-6-b.col(j)).norm()<1e-7,"Control Jacobian");}
        }
        auto c=ddpack::examples::make("planar"); auto s=ddpack::solve(c.problem,c.options);
        check(s.metrics.success,"Planar acceptance");
        check(s.metrics.euler_defect<1e-12 && s.metrics.rk4_defect<1e-12,"Independent linear rollout");
        // An exact full circle: both Euler endpoint nodes are outside a sphere,
        // but the actual held-control motion intersects it at half a turn.
        auto arc=ddpack::examples::make("obstacle_2d");
        arc.problem.x0=ddpack::Vector::Zero(3); arc.problem.target=arc.problem.x0;
        arc.problem.target[2]=2*std::acos(-1.);
        arc.problem.obstacle_centers=ddpack::Matrix::Zero(2,1);arc.problem.obstacle_centers(1,0)=1;
        arc.problem.obstacle_radii[0]=.1;arc.problem.safety_margin=0;
        ddpack::Solution circle; circle.states=ddpack::Matrix::Zero(3,2);
        circle.controls=ddpack::Matrix::Constant(1,2,2);circle.time=ddpack::Vector::Zero(2);circle.time[1]=std::acos(-1.);
        circle.returned_cost=0;circle.initial_cost=1;
        auto arc_metrics=ddpack::compute_metrics(arc.problem,circle);
        // Circle center is (0,0.5), so put the obstacle at the top of the circle.
        arc.problem.obstacle_centers(0,0)=0;arc.problem.obstacle_centers(1,0)=1;
        arc_metrics=ddpack::compute_metrics(arc.problem,circle);
        check(arc_metrics.analytic_zoh_clearance_lower_bound<-.1,"Curved intersample collision not detected");
        check(arc_metrics.analytic_zoh_terminal_error<1e-12,"Exact full-circle endpoint");
        check(!arc_metrics.success,"Colliding curved rollout accepted");
        auto again=ddpack::solve(c.problem,c.options); check((s.states-again.states).norm()==0,"Deterministic solve");
        auto bad=c.problem; bad.dt=bad.tf;
        bool threw=false; try {ddpack::solve(bad,c.options);} catch(const std::invalid_argument&) {threw=true;} check(threw,"Time-grid validation");
        bad=c.problem; bad.u_max[0]=-1; threw=false;
        try {ddpack::solve(bad,c.options);} catch(const std::invalid_argument&) {threw=true;} check(threw,"Bound validation");
        bad=c.problem; bad.x0=ddpack::Vector::Zero(1); bad.target=bad.x0; threw=false;
        try {ddpack::solve(bad,c.options);} catch(const std::invalid_argument&) {threw=true;} check(threw,"Model dimension validation before Eigen operations");
        ddpack::Problem obstacle; obstacle.obstacle_centers=ddpack::Matrix::Zero(2,1); obstacle.obstacle_radii=ddpack::Vector::Constant(1,.5);
        ddpack::Matrix path(2,2); path<<-1,1,0,0;
        check(std::abs(ddpack::minimum_segment_clearance(obstacle,path)+.5)<1e-12,"Segment collision missed by node tests");
        path.col(1)=path.col(0); check(std::abs(ddpack::minimum_segment_clearance(obstacle,path)-.5)<1e-12,"Zero-length segment");
        for(const auto& name:ddpack::examples::names()) {
            c=ddpack::examples::make(name); s=ddpack::solve(c.problem,c.options);
            std::cout<<name<<": "<<(s.metrics.success?"PASS":"FAIL")<<" terminal="<<s.metrics.terminal_error<<" euler="<<s.metrics.euler_defect<<'\n';
            check(s.metrics.success,"Formal example failed");
            if(c.problem.geometric_refinement_iterations>0) {
                check(s.metrics.analytic_zoh_clearance_lower_bound>0,"Analytic continuous clearance bound");
                check(s.metrics.analytic_zoh_terminal_error<c.problem.terminal_tolerance,"Analytic ZOH terminal acceptance");
                check(s.refinement_selected_start>0,"Refinement did not select an improved feasible restart");
                check(s.returned_cost<s.refinement_baseline_cost,"Refinement did not improve the original objective");
                check(s.refinement_rate_cost<s.refinement_baseline_rate_cost,"Refinement did not reduce control-rate energy");
                check(s.final_time>=c.problem.minimum_time && s.final_time<=c.problem.maximum_time,"Refined time violates bounds");
                for(std::size_t k=1;k<s.refinement_penalty_history.size();++k)
                    check(s.refinement_penalty_history[k]<s.refinement_penalty_history[k-1],"Refinement merit was not decreasing");
            }
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
