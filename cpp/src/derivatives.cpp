// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/derivatives.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ddpack {
namespace {
void require(bool condition,const char* message){if(!condition)throw std::invalid_argument(message);}
Matrix value(const std::function<Matrix()>& fn,Eigen::Index rows,Eigen::Index cols){
    Matrix result=fn();
    if(result.rows()!=rows || result.cols()!=cols || !result.allFinite())
        throw std::runtime_error("Callback result has incorrect dimensions or nonfinite entries");
    return result;
}
Matrix difference(const std::function<Vector(const Vector&)>& fn,const Vector& point,int outputs,double step){
    Matrix jac(outputs,point.size());
    for(Eigen::Index j=0;j<point.size();++j){
        const double width=step*std::max(1.,std::abs(point[j]));Vector plus=point,minus=point;
        plus[j]+=width;minus[j]-=width;
        Matrix a=value([&](){return fn(plus);},outputs,1),b=value([&](){return fn(minus);},outputs,1);
        jac.col(j)=(a-b)/(2*width);
    }
    return jac;
}
}
DerivativeReport check_derivatives(const Problem& p,const Vector& x,const Vector& u,const DerivativeCheckOptions& o){
    const int nx=static_cast<int>(x.size()),nu=static_cast<int>(u.size());
    require(nx>0 && nu>0 && x.allFinite() && u.allFinite() && p.target.size()==nx && p.target.allFinite(),"Invalid derivative-check point");
    require(bool(p.dynamics) && bool(p.cost),"Derivative check requires dynamics and cost");
    for(double option:{o.difference_step,o.absolute_tolerance,o.relative_tolerance})
        require(std::isfinite(option) && option>0,"Derivative-check options must be finite and positive");
    if(const auto* model=dynamic_cast<const LinearDynamics*>(p.dynamics.get()))
        require(model->A.cols()==nx && model->B.cols()==nu,"Linear-model dimensions differ from derivative-check point");
    if(dynamic_cast<const PendulumDynamics*>(p.dynamics.get()))require(nx==2 && nu==1,"Pendulum point dimensions");
    if(dynamic_cast<const PlanarDynamics*>(p.dynamics.get()))require(nx==3 && nu==1,"Planar point dimensions");
    if(dynamic_cast<const FlightDynamics*>(p.dynamics.get()))require(nx==6 && nu==2,"Flight point dimensions");
    if(dynamic_cast<const ThermalDynamics*>(p.dynamics.get()))require(nx==1 && nu==1,"Thermal point dimensions");
    if(const auto* cost=dynamic_cast<const QuadraticCost*>(p.cost.get()))
        require(cost->Qf.rows()==nx && cost->Qf.cols()==nx && cost->Q.rows()==nx && cost->Q.cols()==nx &&
                cost->R.rows()==nu && cost->R.cols()==nu,"Cost dimensions differ from derivative-check point");
    DerivativeReport report;
    report.options=o;report.state=x;report.control=u;
    auto add=[&](const std::string& name,const std::function<Matrix()>& analytic,
                 const std::function<Matrix()>& numerical,int rows,int cols){
        DerivativeCheck check;check.name=name;
        try{
            Matrix a=value(analytic,rows,cols),b=value(numerical,rows,cols);
            Matrix delta=(a-b).cwiseAbs(),scale=a.cwiseAbs().cwiseMax(b.cwiseAbs());
            check.max_absolute_error=delta.maxCoeff();
            check.max_scaled_error=(delta.array()/scale.array().max(1.)).maxCoeff();
            check.max_tolerance_ratio=(delta.array()/(o.absolute_tolerance+o.relative_tolerance*scale.array())).maxCoeff();
            check.passed=check.max_tolerance_ratio<=1;
            if(!check.passed)check.message="Analytic derivative differs from independent reference";
        }catch(const std::exception& error){check.message=error.what();}
        report.checks.push_back(check);
    };
    auto contract=[&](const std::string& name,const std::function<Matrix()>& callback,int rows,int cols){
        DerivativeCheck check;check.name=name;check.category="value_contract";
        check.max_absolute_error=check.max_scaled_error=check.max_tolerance_ratio=0;
        try{value(callback,rows,cols);check.passed=true;}
        catch(const std::exception& error){check.message=error.what();}
        report.checks.push_back(check);
    };
    const auto& d=*p.dynamics;const auto& c=*p.cost;
    contract("dynamics.f",[&](){return d.f(x,u);},nx,1);
    contract("terminal.value",[&](){return Matrix::Constant(1,1,c.terminal(x,p.target));},1,1);
    contract("running.value",[&](){return Matrix::Constant(1,1,c.running(x,p.target,u));},1,1);
    auto dx=[&](const Vector& v){return difference([&](const Vector& point){return d.f(point,u);},v,nx,o.difference_step);};
    add("dynamics.fx",[&](){return d.fx(x,u);},[&](){return dx(x);},nx,nx);
    add("dynamics.fu",[&](){return d.fu(x,u);},[&](){return difference([&](const Vector& v){return d.f(x,v);},u,nx,o.difference_step);},nx,nu);
    add("terminal.x",[&](){return c.terminal_x(x,p.target);},[&](){return difference([&](const Vector& v){return Vector::Constant(1,c.terminal(v,p.target));},x,1,o.difference_step).transpose().eval();},nx,1);
    add("terminal.xx",[&](){return c.terminal_xx(x,p.target);},[&](){return difference([&](const Vector& v){return c.terminal_x(v,p.target);},x,nx,o.difference_step);},nx,nx);
    add("terminal.symmetry",[&](){return c.terminal_xx(x,p.target);},[&](){return c.terminal_xx(x,p.target).transpose().eval();},nx,nx);
    add("running.x",[&](){return c.running_x(x,p.target,u);},[&](){return difference([&](const Vector& v){return Vector::Constant(1,c.running(v,p.target,u));},x,1,o.difference_step).transpose().eval();},nx,1);
    add("running.u",[&](){return c.running_u(x,p.target,u);},[&](){return difference([&](const Vector& v){return Vector::Constant(1,c.running(x,p.target,v));},u,1,o.difference_step).transpose().eval();},nu,1);
    add("running.xx",[&](){return c.running_xx(x,p.target,u);},[&](){return difference([&](const Vector& v){return c.running_x(v,p.target,u);},x,nx,o.difference_step);},nx,nx);
    add("running.uu",[&](){return c.running_uu(x,p.target,u);},[&](){return difference([&](const Vector& v){return c.running_u(x,p.target,v);},u,nu,o.difference_step);},nu,nu);
    add("running.xu",[&](){return c.running_xu(x,p.target,u);},[&](){return difference([&](const Vector& v){return c.running_x(x,p.target,v);},u,nx,o.difference_step);},nx,nu);
    add("running.ux",[&](){return c.running_xu(x,p.target,u).transpose().eval();},[&](){return difference([&](const Vector& v){return c.running_u(v,p.target,u);},x,nu,o.difference_step);},nu,nx);
    add("running.state_symmetry",[&](){return c.running_xx(x,p.target,u);},[&](){return c.running_xx(x,p.target,u).transpose().eval();},nx,nx);
    add("running.control_symmetry",[&](){return c.running_uu(x,p.target,u);},[&](){return c.running_uu(x,p.target,u).transpose().eval();},nu,nu);
    for(std::size_t j=0;j<p.constraints.size();++j){
        const auto& constraint=p.constraints[j];
        contract("constraint."+std::to_string(j+1)+".g",[&](){return Matrix::Constant(1,1,constraint.g(x,u));},1,1);
        add("constraint."+std::to_string(j+1)+".gx",[&](){return constraint.gx(x,u);},[&](){return difference([&](const Vector& v){return Vector::Constant(1,constraint.g(v,u));},x,1,o.difference_step).transpose().eval();},nx,1);
    }
    report.success=std::all_of(report.checks.begin(),report.checks.end(),[](const DerivativeCheck& check){return check.passed;});
    return report;
}
}
