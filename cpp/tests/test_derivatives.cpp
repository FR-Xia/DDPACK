// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/derivatives.hpp>
#include "cli_arguments.hpp"
#include "problems.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace ddpack;
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
struct BrokenDynamics final:Dynamics {
    LinearDynamics model{Matrix::Zero(2,2),Matrix::Ones(2,1),Vector::Zero(2)};int mode=0;
    Vector f(const Vector& x,const Vector& u)const override{
        if(mode==4 && (x.array()==.2).all() && u[0]==.1)return Vector::Constant(2,infinity);
        return model.f(x,u);
    }
    Matrix fx(const Vector& x,const Vector& u)const override{
        if(mode==1)return Matrix::Zero(1,2);
        if(mode==2)return Matrix::Constant(2,2,infinity);
        if(mode==3)throw std::runtime_error("Deliberate derivative callback failure");
        return mode==4?model.fx(x,u):Matrix(model.fx(x,u)+.1*Matrix::Identity(2,2));
    }
    Matrix fu(const Vector& x,const Vector& u)const override{return model.fu(x,u);}
};
struct MixedCost final:Cost {
    QuadraticCost q{Matrix::Identity(2,2),3*Matrix::Identity(2,2),2*Matrix::Identity(2,2)};
    Matrix s;bool wrong_cross=false;
    MixedCost(){s.resize(2,2);s<<.2,-.3,.4,.1;}
    double terminal(const Vector& x,const Vector& t)const override{return q.terminal(x,t);}
    Vector terminal_x(const Vector& x,const Vector& t)const override{return q.terminal_x(x,t);}
    Matrix terminal_xx(const Vector& x,const Vector& t)const override{return q.terminal_xx(x,t);}
    double running(const Vector& x,const Vector& t,const Vector& u)const override{return q.running(x,t,u)+(x-t).dot(s*u);}
    Vector running_x(const Vector& x,const Vector& t,const Vector& u)const override{return q.running_x(x,t,u)+s*u;}
    Vector running_u(const Vector& x,const Vector& t,const Vector& u)const override{return q.running_u(x,t,u)+s.transpose()*(x-t);}
    Matrix running_xx(const Vector& x,const Vector& t,const Vector& u)const override{return q.running_xx(x,t,u);}
    Matrix running_uu(const Vector& x,const Vector& t,const Vector& u)const override{return q.running_uu(x,t,u);}
    Matrix running_xu(const Vector&,const Vector&,const Vector&)const override{return wrong_cross?Matrix::Zero(2,2):s;}
};
int DDPACK_CLI_MAIN(int argc,cli::argument_char** argv){
    try{
        check(argc<=2,"Usage: ddpack_derivative_tests [fresh_output_directory]");
        auto args=cli::arguments(argc,argv);std::ofstream csv,points;
        if(argc==2){
            auto path=std::filesystem::u8path(args[1]);check(!std::filesystem::exists(path),"Audit output already exists");
            std::filesystem::create_directories(path);csv.open(path/"derivative_checks.csv");points.open(path/"points.json");
            csv.exceptions(std::ios::badbit|std::ios::failbit);points.exceptions(std::ios::badbit|std::ios::failbit);
            csv<<std::setprecision(17)<<"case,sample,difference_step,check,passed,max_absolute_error,max_scaled_error,max_tolerance_ratio,category\n";
            points<<std::setprecision(17)<<"[\n";
        }
        int reports=0,checks=0;double maximum=0;
        for(const auto& name:examples::names()){
            auto c=examples::make(name);
            for(int sample=1;sample<=3;++sample){
                Vector x=(1-.25*sample)*c.problem.x0+.25*sample*c.problem.target,u(c.problem.u_max.size());
                for(Eigen::Index j=0;j<x.size();++j)x[j]+=.07*std::sin(double(j+1+sample));
                for(Eigen::Index j=0;j<u.size();++j)u[j]=.13*sample*c.problem.u_max[j]*std::cos(double(j+1+2*sample));
                for(double width:{1e-4,1e-5,1e-6}){
                    DerivativeCheckOptions options;options.difference_step=width;
                    auto report=check_derivatives(c.problem,x,u,options);
                    for(const auto& row:report.checks){
                        if(!row.passed)std::cerr<<name<<' '<<sample<<' '<<width<<' '<<row.name<<": "<<row.message<<'\n';
                        check(row.passed,"Formal derivative contract failed");++checks;maximum=std::max(maximum,row.max_tolerance_ratio);
                        if(csv.is_open())csv<<name<<','<<sample<<','<<width<<','<<row.name<<','<<row.passed<<','<<row.max_absolute_error<<','<<row.max_scaled_error<<','<<row.max_tolerance_ratio<<','<<row.category<<'\n';
                    }
                    if(points.is_open()){
                        if(reports)points<<",\n";points<<"{\"case\":\""<<name<<"\",\"sample\":"<<sample<<",\"difference_step\":"<<width<<",\"state\":[";
                        for(Eigen::Index j=0;j<x.size();++j){if(j)points<<',';points<<x[j];}
                        points<<"],\"control\":[";for(Eigen::Index j=0;j<u.size();++j){if(j)points<<',';points<<u[j];}points<<"]}";
                    }
                    check(report.success,"Aggregate derivative result differs");++reports;
                }
            }
        }
        if(points.is_open())points<<"\n]\n";
        Problem p;p.x0=Vector::Zero(2);p.target=p.x0;p.u_max=Vector::Ones(1);
        p.cost=std::make_shared<QuadraticCost>(Matrix::Identity(2,2),Matrix::Identity(2,2),Matrix::Ones(1,1));
        auto broken=std::make_shared<BrokenDynamics>();p.dynamics=broken;
        for(int mode=0;mode<5;++mode){broken->mode=mode;check(!check_derivatives(p,Vector::Constant(2,.2),Vector::Constant(1,.1)).success,"Broken derivative escaped detection");}
        p.u_max=Vector::Ones(2);p.dynamics=std::make_shared<LinearDynamics>(Matrix::Identity(2,2),Matrix::Identity(2,2),Vector::Zero(2));
        auto mixed=std::make_shared<MixedCost>();p.cost=mixed;Vector x(2),u(2);x<<.3,-.4;u<<.1,-.2;
        check(check_derivatives(p,x,u).success,"Nonseparable cost derivatives failed");
        mixed->wrong_cross=true;check(!check_derivatives(p,x,u).success,"Wrong mixed Hessian escaped detection");mixed->wrong_cross=false;
        p.constraints.push_back(state_lower_bound(0,.1,2));
        for(double position:{-.4,.1,.6}){x[0]=position;check(check_derivatives(p,x,u).success,"Constraint derivative sign/shape");}
        p.constraints[0].gx=[](const Vector&,const Vector&){return Vector::Zero(2);};
        check(!check_derivatives(p,x,u).success,"Wrong constraint derivative escaped detection");
        DerivativeCheckOptions invalid;invalid.difference_step=0;bool threw=false;
        try{check_derivatives(p,x,u,invalid);}catch(const std::invalid_argument&){threw=true;}check(threw,"Invalid stencil accepted");
        if(argc==2){
            std::ofstream summary(std::filesystem::u8path(args[1])/"summary.json");
            summary<<std::setprecision(17)<<"{\"success\":true,\"formal_cases\":9,\"reports\":"<<reports<<",\"checks\":"<<checks<<",\"failed_checks\":0,\"maximum_tolerance_ratio\":"<<maximum<<",\"counterexample_checks_passed\":true}\n";
        }
        std::cout<<reports<<" native derivative probes, "<<checks<<" checks, counterexamples PASS\n";return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
