// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/ddpack.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ddpack {
// Separate bounded-time stage: bounded control knots, exact rollout, and
// analytic shooting Jacobians. No state/output smoothing is performed.
void refine_geometric(const Problem& p,Solution& s) {
    if(p.geometric_refinement_iterations==0) return;
    const auto* cost=dynamic_cast<const QuadraticCost*>(p.cost.get());
    if(!cost) throw std::invalid_argument("Geometric refinement requires QuadraticCost");
    const int nx=static_cast<int>(p.x0.size()),nu=static_cast<int>(p.u_max.size());
    const int n=static_cast<int>(s.states.cols()),m=n-1,knot=std::min(p.geometric_refinement_knots,m),controlsCount=nu*knot,nv=controlsCount+(p.geometric_refinement_free_time?1:0);
    const bool freeTime=p.geometric_refinement_free_time;
    const double originalTime=s.final_time,lambda=p.geometric_control_rate_weight;
    const int dim=static_cast<int>(p.obstacle_centers.rows()),obs=static_cast<int>(p.obstacle_centers.cols());
    Matrix basis=Matrix::Zero(m,knot);
    for(int k=0;k<m;++k) {
        const double z=double(k)*(knot-1)/(m-1);const int i=std::min(static_cast<int>(z),knot-2);
        basis(k,i)=1-(z-i);basis(k,i+1)=z-i;
    }
    std::vector<Matrix> maps(m,Matrix::Zero(nu,nv));
    for(int k=0;k<m;++k) for(int j=0;j<knot;++j) maps[k].block(0,j*nu,nu,nu)=basis(k,j)*Matrix::Identity(nu,nu);
    Matrix q=(cost->Q+1e-14*Matrix::Identity(nx,nx)).llt().matrixU();
    Matrix r=(cost->R+1e-14*Matrix::Identity(nu,nu)).llt().matrixU();
    Matrix f=(cost->Qf+1e5*Matrix::Identity(nx,nx)).llt().matrixU();
    auto bound=[&](Matrix z) {for(int j=0;j<z.cols();++j) z.col(j)=z.col(j).cwiseMax(-p.u_max).cwiseMin(p.u_max);return z;};
    struct Evaluation {Matrix x,jac;Vector residual;double j=0,smooth=0,clear=infinity;};
    auto evaluate=[&](const Matrix& u,double time,bool derivatives) {
        const double h=time/m;
        Evaluation e;e.x=Matrix::Zero(nx,n);e.x.col(0)=p.x0;
        std::vector<Matrix> sx(n,Matrix::Zero(nx,nv));
        for(int k=0;k<m;++k) {
            if(derivatives) { sx[k+1]=(Matrix::Identity(nx,nx)+h*p.dynamics->fx(e.x.col(k),u.col(k)))*sx[k]+h*p.dynamics->fu(e.x.col(k),u.col(k))*maps[k];
                if(freeTime)sx[k+1].col(nv-1)+=p.dynamics->f(e.x.col(k),u.col(k))/m; }
            e.x.col(k+1)=e.x.col(k)+h*p.dynamics->f(e.x.col(k),u.col(k));
        }
        const int nr=nx*m+nu*m+nu*(m-1)+nx+obs*m;
        e.residual=Vector::Zero(nr);if(derivatives)e.jac=Matrix::Zero(nr,nv);
        int row=0;
        for(int k=0;k<m;++k) {
            Vector v=std::sqrt(h)*q*(e.x.col(k)-p.target);e.j+=.5*v.squaredNorm();
            e.residual.segment(row,nx)=v;if(derivatives)e.jac.middleRows(row,nx)=std::sqrt(h)*q*sx[k];if(derivatives && freeTime)e.jac.block(row,nv-1,nx,1)+=v/(2*time);row+=nx;
        }
        for(int k=0;k<m;++k) {
            Vector v=std::sqrt(h)*r*u.col(k);e.j+=.5*v.squaredNorm();
            e.residual.segment(row,nu)=v;if(derivatives)e.jac.middleRows(row,nu)=std::sqrt(h)*r*maps[k];if(derivatives && freeTime)e.jac.block(row,nv-1,nu,1)+=v/(2*time);row+=nu;
        }
        for(int k=0;k<m-1;++k) {
            Vector v=std::sqrt(lambda/h)*(u.col(k+1)-u.col(k));e.smooth+=.5*v.squaredNorm();
            e.residual.segment(row,nu)=v;if(derivatives)e.jac.middleRows(row,nu)=std::sqrt(lambda/h)*(maps[k+1]-maps[k]);if(derivatives && freeTime)e.jac.block(row,nv-1,nu,1)-=v/(2*time);row+=nu;
        }
        Vector terminal=e.x.col(n-1)-p.target;e.j+=.5*terminal.dot(cost->Qf*terminal);
        e.residual.segment(row,nx)=f*terminal;if(derivatives)e.jac.middleRows(row,nx)=f*sx[n-1];row+=nx;
        for(int ob=0;ob<obs;++ob) for(int k=0;k<m;++k) {
            Vector offset=e.x.col(k).head(dim)-p.obstacle_centers.col(ob),delta=e.x.col(k+1).head(dim)-e.x.col(k).head(dim);
            const double fraction=std::clamp(-offset.dot(delta)/std::max(delta.squaredNorm(),1e-30),0.,1.);
            Vector closest=offset+fraction*delta;double distance=closest.norm(),gap=distance-p.obstacle_radii[ob]-p.safety_margin;
            e.clear=std::min(e.clear,gap);e.residual[row]=std::sqrt(1e5)*std::max(0.,.005-gap);
            if(derivatives && gap<.005 && distance>1e-12) e.jac.row(row)=-std::sqrt(1e5)*closest.transpose()/distance*((1-fraction)*sx[k].topRows(dim)+fraction*sx[k+1].topRows(dim));
            ++row;
        }
        return e;
    };
    // Canonical restarts are independent of the legacy trajectory's side.
    std::vector<Matrix> seeds;
    for(int axis=0;axis<nu;++axis) for(double sign:{1.,-1.}) {
        Matrix z=Matrix::Zero(nu,knot);
        for(int j=0;j<knot;++j)z(axis,j)=sign*.4*p.u_max[axis]*std::cos(2*std::acos(-1.)*j/(knot-1));
        seeds.push_back(z);
    }
    auto base=evaluate(s.controls.leftCols(m),originalTime,false);double bestScore=base.j+base.smooth,bestTime=originalTime;
    const double initialObjective=std::isfinite(s.initial_cost)?s.initial_cost:base.j;
    auto physicallyFeasible=[&](const Evaluation& e,const Matrix& control,double time) {
        Problem checkProblem=p;checkProblem.require_cost_decrease=false;
        Solution candidate;candidate.states=e.x;candidate.controls.resize(nu,n);
        candidate.controls.leftCols(m)=control;candidate.controls.col(m)=control.col(m-1);
        candidate.time=Vector::LinSpaced(n,0,time);candidate.final_time=time;
        candidate.returned_cost=e.j;candidate.initial_cost=e.j;
        return compute_metrics(checkProblem,candidate).numerical_checks_pass;
    };
    bool bestEligible=physicallyFeasible(base,s.controls.leftCols(m),originalTime) &&
        (!p.require_cost_decrease || base.j<=initialObjective*(1+1e-10));
    Matrix bestU=s.controls.leftCols(m),bestX=base.x;
    s.refinement_baseline_cost=base.j;s.refinement_baseline_rate_cost=base.smooth;s.refinement_rate_cost=base.smooth;
    s.refinement_starts=Matrix::Zero(seeds.size(),7);
    for(int start=0;start<static_cast<int>(seeds.size());++start) {
        Vector z(nv);z.head(controlsCount)=Eigen::Map<const Vector>(seeds[start].data(),controlsCount);
        if(freeTime)z[nv-1]=std::min(p.maximum_time,std::max(originalTime,p.tf*1.04));
        auto clampVariables=[&](Vector v) {Matrix controls=bound(Eigen::Map<const Matrix>(v.data(),nu,knot));v.head(controlsCount)=Eigen::Map<const Vector>(controls.data(),controlsCount);if(freeTime)v[nv-1]=std::clamp(v[nv-1],p.minimum_time,p.maximum_time);return v;};
        auto controls=[&](const Vector& v)->Matrix {return Eigen::Map<const Matrix>(v.data(),nu,knot)*basis.transpose();};
        auto time=[&](const Vector& v){return freeTime?v[nv-1]:originalTime;};
        z=clampVariables(z);double damping=1e-3;std::vector<double> history;int iteration=0;
        for(;iteration<p.geometric_refinement_iterations;++iteration) {
            auto e=evaluate(controls(z),time(z),true);const double merit=.5*e.residual.squaredNorm();history.push_back(merit);
            Vector gradient=e.jac.transpose()*e.residual;
            Vector projected=z-clampVariables(z-gradient);
            if(projected.cwiseAbs().maxCoeff()<1e-7) break;
            Matrix normal=e.jac.transpose()*e.jac;bool accepted=false;
            for(int retry=0;retry<12;++retry) {
                Matrix damped=normal;damped.diagonal().array()+=damping*normal.diagonal().cwiseMax(1.).array();
                Vector direction=-damped.llt().solve(gradient);
                Vector candidate=clampVariables(z+direction);
                auto next=evaluate(controls(candidate),time(candidate),false);double value=.5*next.residual.squaredNorm();
                if(std::isfinite(value) && value<merit-1e-12*std::max(1.,merit)) {z=candidate;damping=std::max(1e-5,damping/3);accepted=true;break;}
                damping*=10;
            }
            if(!accepted) break;
        }
        auto e=evaluate(controls(z),time(z),false);double error=(e.x.col(n-1)-p.target).norm();
        bool feasible=physicallyFeasible(e,controls(z),time(z));
        s.refinement_starts.row(start)<<start+1,std::min(iteration+1,p.geometric_refinement_iterations),e.j,e.smooth,error,e.clear,feasible?1:0;
        const bool costAllowed=!p.require_cost_decrease || e.j<=initialObjective*(1+1e-10);
        if(feasible && costAllowed && (!bestEligible || e.j+e.smooth<bestScore-1e-8*std::max(1.,std::abs(bestScore)))) {
            bestScore=e.j+e.smooth;bestTime=time(z);bestU=controls(z);bestX=e.x;
            bestEligible=true;
            s.refinement_selected_start=start+1;s.refinement_penalty_history=history;s.refinement_rate_cost=e.smooth;
        }
    }
    s.states=bestX;s.final_time=bestTime;s.controls.leftCols(m)=bestU;s.controls.col(m)=bestU.col(m-1);
    if(s.refinement_selected_start>0)s.termination_reason="geometric_refinement_feasible";
}
} // namespace ddpack
