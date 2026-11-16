// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include "line_search.hpp"
#include "problems.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct DomainDynamics final:ddpack::Dynamics {
    ddpack::Vector f(const ddpack::Vector&,const ddpack::Vector& u)const override{return ddpack::Vector::Constant(1,u[0]>0?ddpack::infinity:0);}
    ddpack::Matrix fx(const ddpack::Vector&,const ddpack::Vector&)const override{return ddpack::Matrix::Zero(1,1);}
    ddpack::Matrix fu(const ddpack::Vector&,const ddpack::Vector&)const override{return ddpack::Matrix::Ones(1,1);}
};
int main(){
    try{
        using namespace ddpack;
        Problem p;p.x0=Vector::Zero(1);p.target=Vector::Ones(1);p.u_max=Vector::Constant(1,10);
        p.dynamics=std::make_shared<LinearDynamics>(Matrix::Zero(1,1),Matrix::Ones(1,1),Vector::Zero(1));
        p.cost=std::make_shared<QuadraticCost>(Matrix::Ones(1,1),Matrix::Zero(1,1),Matrix::Constant(1,1,.1));
        Options o;o.algorithm=Algorithm::terminal_defect_pddp;o.epsilon=1;
        Matrix x=Matrix::Zero(1,3),u=x;
        std::vector<Vector> feed(3,Vector::Constant(1,8)),timegain(3,Vector::Zero(1));
        std::vector<Matrix> feedback(3,Matrix::Zero(1,1));
        auto trial=defect_line_search(p,o,x,u,1,.5,2,0,feed,feedback,timegain,10,1);
        check(trial.accepted && trial.trials.size()==4,"Overshoot was not backtracked");
        check(trial.trials.back()[2]==.125 && std::abs(trial.cost-.05)<1e-14,"Accepted merit not recomputed");
        check(std::abs(trial.states(0,2)-1)<1e-14,"Accepted trajectory mismatch");
        p.target.setZero();feed.assign(3,Vector::Ones(1));
        trial=defect_line_search(p,o,x,u,1,.5,1.1,1,feed,feedback,timegain,10,1);
        check(!trial.accepted && trial.trials.size()==24,"Increasing candidates were accepted");
        check((trial.states-x).norm()==0 && (trial.controls-u).norm()==0 && trial.time==1,"Rejected trial polluted nominal");
        for(const auto& row:trial.trials)check(row[5]<=1.1 && row.back()==0,"Bound or rejection record mismatch");
        p.dynamics=std::make_shared<DomainDynamics>();
        trial=defect_line_search(p,o,x,u,1,.5,2,0,feed,feedback,timegain,10,1);
        check(!trial.accepted && (trial.states-x).norm()==0,"Nonfinite candidate escaped rollback");
        for(const auto& row:trial.trials)check(row[row.size()-2]==0 && row.back()==0,"Nonfinite trial accepted");
        for(const auto& name:std::vector<std::string>{"terminal_defect","obstacle_2d","obstacle_3d"}){
            auto c=examples::make(name);c.options.max_iterations=1;
            c.problem.projection_max_iterations=0;c.problem.geometric_refinement_iterations=0;
            auto s=solve(c.problem,c.options);check(!s.line_search_trials.empty(),"Missing actual merit trials");
            for(const auto& row:s.line_search_trials){
                check(std::abs(row[11]-(row[7]+row[6]*row[9]+row[15]*row[13]))<1e-8,"Nominal merit formula");
                if(row.back())check(row[row.size()-2] && row[12]<row[11],"Accepted trial has no real decrease");
            }
            check(s.time_history.size()==s.trial_costs.size(),"Time/cost histories not aligned");
            check(std::abs(s.returned_cost-s.trial_costs.back())<1e-9,"Returned cost lags the accepted trajectory");
            check(s.metrics.euler_defect<1e-12,"Returned trajectory not reproducible");
        }
        std::cout<<"Actual-merit backtracking and rollback PASS\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
