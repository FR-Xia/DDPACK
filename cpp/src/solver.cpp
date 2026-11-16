// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/ddpack.hpp>
#include "line_search.hpp"
#include "numerics.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace ddpack {
void refine_geometric(const Problem&,Solution&);
namespace {
bool geometric(Algorithm a) { return a==Algorithm::geometric_safety_pddp_2d || a==Algorithm::geometric_safety_pddp_3d; }
void require(bool condition, const char* message) { if(!condition) throw std::invalid_argument(message); }
Vector clamp(const Vector& u, const Vector& b) { return u.cwiseMax(-b).cwiseMin(b); }
void validate(const Problem& p, const Options& o) {
    algorithm_name(o.algorithm);
    require(p.x0.size()>0 && p.target.size()==p.x0.size() && p.x0.allFinite() && p.target.allFinite(),"Invalid states");
    require(p.u_max.size()>0 && p.u_max.allFinite() && (p.u_max.array()>0).all(),"Invalid control bounds");
    require(p.dynamics && p.cost,"Dynamics and cost are required");
    require(std::isfinite(p.tf) && std::isfinite(p.dt) && p.dt>0 && p.tf>p.dt && p.tf/p.dt<1000000,"Invalid time grid");
    require(p.intervals>=2 && p.intervals<1000000 && o.max_iterations>0,"Invalid iteration or node count");
    require(std::isfinite(o.alpha) && o.alpha>0 && o.alpha<=1 && std::isfinite(o.epsilon) && o.epsilon>0 &&
        std::isfinite(o.rho) && o.rho>0 && o.rho<1 && std::isfinite(o.update_epsilon) && o.update_epsilon>0,"Invalid solver options");
    for(double v : {p.terminal_tolerance,p.constraint_tolerance,p.defect_tolerance,p.safety_margin,p.enforcement_margin,p.safety_gain,p.projection_tolerance,p.minimum_time,p.maximum_time})
        require(std::isfinite(v) && v>=0,"Invalid tolerance, margin or time bound");
    require(!std::isnan(p.rk4_tolerance) && p.rk4_tolerance>=0 && p.projection_max_iterations>=0,"Invalid diagnostic/projection options");
    require(p.geometric_refinement_iterations>=0 && p.geometric_refinement_knots>=3 && std::isfinite(p.geometric_control_rate_weight) && p.geometric_control_rate_weight>=0,"Invalid refinement options");
    if(p.geometric_refinement_iterations>0) {
        require(geometric(o.algorithm) && std::floor(p.tf/p.dt)>=4,"Refinement requires geometric mode and at least four nodes");
        require(!p.geometric_refinement_free_time || (p.minimum_time>0 && p.maximum_time>p.minimum_time),"Free-time refinement requires positive ordered time bounds");
    }
    require(p.maximum_time==0 || p.minimum_time<p.maximum_time,"Invalid time bounds");
    require(geometric(o.algorithm) || (p.minimum_time==0 && p.maximum_time==0),
        "minimum_time and maximum_time are supported only by geometric algorithms");
    if(geometric(o.algorithm)) {
        const double lower=p.minimum_time>0?p.minimum_time:std::max(p.tf-20,5.);
        const double upper=p.maximum_time>0?p.maximum_time:p.tf+7;
        require(lower<upper,"Invalid effective geometric time bounds");
    }
    const Vector u=Vector::Zero(p.u_max.size());
    if(const auto* d=dynamic_cast<const LinearDynamics*>(p.dynamics.get()))
        require(d->A.cols()==p.x0.size() && d->B.cols()==u.size(),"Linear model dimensions do not match problem");
    if(const auto* d=dynamic_cast<const PendulumDynamics*>(p.dynamics.get())) {
        require(p.x0.size()==2 && u.size()==1,"Pendulum requires two states and one control");
        require(std::isfinite(d->mass) && d->mass>0 && std::isfinite(d->length) && d->length>0 && std::isfinite(d->gravity) && std::isfinite(d->damping),"Invalid pendulum parameters");
    }
    if(const auto* d=dynamic_cast<const PlanarDynamics*>(p.dynamics.get()))
        require(p.x0.size()==3 && u.size()==1 && std::isfinite(d->speed) && d->speed>0,"Invalid planar model");
    if(dynamic_cast<const FlightDynamics*>(p.dynamics.get())) require(p.x0.size()==6 && u.size()==2,"Flight model requires six states and two controls");
    if(dynamic_cast<const ThermalDynamics*>(p.dynamics.get())) require(p.x0.size()==1 && u.size()==1,"Thermal model requires one state and one control");
    require(p.constraints.empty() || o.algorithm==Algorithm::free_time_pddp || o.algorithm==Algorithm::penalty_free_time_ddp,
        "State-constraint callbacks apply only to free_time_pddp and penalty_free_time_ddp; use obstacle geometry for safety variants");
    Vector f=p.dynamics->f(p.x0,u); Matrix a=p.dynamics->fx(p.x0,u),b=p.dynamics->fu(p.x0,u);
    require(f.size()==p.x0.size() && a.rows()==p.x0.size() && a.cols()==p.x0.size() && b.rows()==p.x0.size() && b.cols()==u.size(),"Dynamics dimensions do not match problem");
    require(f.allFinite() && a.allFinite() && b.allFinite(),"Nonfinite dynamics");
    auto q=dynamic_cast<const QuadraticCost*>(p.cost.get());
    if(q) require(q->Q.rows()==p.x0.size() && q->R.rows()==u.size(),"Cost dimensions do not match problem");
    for(const auto& c:p.constraints) {
        require(bool(c.g) && bool(c.gx),"Incomplete constraint");
        Vector g=c.gx(p.x0,u); require(g.size()==p.x0.size() && g.allFinite() && std::isfinite(c.g(p.x0,u)),"Invalid constraint");
    }
    if(geometric(o.algorithm)) {
        const int d=o.algorithm==Algorithm::geometric_safety_pddp_2d?2:3;
        require(p.obstacle_centers.rows()==d && p.obstacle_centers.cols()>0 && p.obstacle_centers.allFinite() && p.x0.size()>=d,"Invalid obstacle centers");
        require(p.obstacle_radii.size()==p.obstacle_centers.cols() && p.obstacle_radii.allFinite() && (p.obstacle_radii.array()>0).all(),"Invalid obstacle radii");
        require((d==2 && p.x0.size()==3 && u.size()==1) || (d==3 && p.x0.size()==6 && u.size()==2),"Geometric guidance requires the documented planar or flight state layout");
    }
    if(o.algorithm==Algorithm::terminal_defect_pddp)
        require(p.x0.size()==3 && u.size()==1,"Terminal-defect guidance requires three states and one control");
    require(p.waypoints.size()==0 || (o.algorithm==Algorithm::waypoint_pddp && p.waypoints.rows()==2 && p.x0.size()>=2 && p.waypoints.allFinite()),"Invalid waypoints");
}
// Preserve the legacy guidance seed, including its fixed physical 0.1 s step.
void guidance(const Problem& p, int n, bool flight, Matrix& x, Matrix& u) {
    const int count=static_cast<int>(std::floor(p.tf/.1));
    require(count>=n,"Guidance seed is shorter than the requested grid (legacy dt must be >= 0.1)");
    Vector pos=p.x0;
    if(!flight) {
        Vector vel=Vector::Zero(3); double turn=0;
        for(int k=0;k<n;++k) {
            x.col(k)=pos; u(0,k)=5*turn;
            if(k==count-1) break;
            const double dx=p.target[0]-pos[0],dy=p.target[1]-pos[1],den=dx*dx+dy*dy;
            if(den==0) throw std::runtime_error("Guidance is singular at the target");
            turn=(-vel[1]*dx+dy*vel[0])/den;
            pos+=.1*vel;
            vel << std::cos(pos[2]),std::sin(pos[2]),5*turn;
        }
    } else {
        Vector vel=Vector::Zero(6), delta=p.target.head(3)-p.x0.head(3);
        const double dist=delta.norm(); if(dist>0) vel.head(3)=std::min(dist/p.tf,50.0)*delta/dist;
        for(int k=0;k<n;++k) {
            x.col(k)=pos;
            if(k==count-1) {u.col(k).setZero(); break;}
            Vector e=.1*(p.target-pos);
            u(0,k)=e[0]; u(1,k)=e[1];
            pos.head(3)+=.1*vel.head(3); pos[4]+=.1*e[4]; pos[5]+=.1*e[5];
            vel.head(3)+=.1*e.head(3);
            if(vel.head(3).norm()>100) vel.head(3)*=100/vel.head(3).norm();
        }
    }
}
Matrix rollout(const Problem& p, const Matrix& u, double tf) {
    Matrix x(p.x0.size(),u.cols()); x.col(0)=p.x0;
    const double step=tf/double(u.cols()-1);
    for(Eigen::Index k=0;k<u.cols()-1;++k) x.col(k+1)=x.col(k)+step*p.dynamics->f(x.col(k),u.col(k));
    return x;
}
double objective(const Problem& p,const Matrix& x,const Matrix& u,double tf,bool penalty=false) {
    double value=p.cost->terminal(x.col(x.cols()-1),p.target),h=tf/double(x.cols()-1);
    for(Eigen::Index k=0;k<x.cols()-1;++k) value+=h*(p.cost->running(x.col(k),p.target,u.col(k))+(penalty?1:0));
    return value;
}
void project_terminal(const Problem& p, Solution& s) {
    if(p.projection_tolerance<=0 || p.projection_max_iterations==0) return;
    Matrix u=s.controls, x=rollout(p,u,s.final_time);
    const Vector scale=(p.target-p.x0).cwiseAbs().cwiseMax(1.0);
    double damping=1e-4;
    for(int it=0;it<p.projection_max_iterations;++it) {
        Vector error=x.col(x.cols()-1)-p.target;
        if(error.norm()<=p.projection_tolerance) break;
        Vector residual=error.cwiseQuotient(scale);
        const int variables=static_cast<int>((u.cols()-1)*u.rows());
        Matrix sensitivity(p.x0.size(),variables);
        for(int j=0;j<variables;++j) {
            Matrix v=u; v(j%u.rows(),j/u.rows())+=1e-5;
            Matrix y=rollout(p,v,s.final_time);
            sensitivity.col(j)=(y.col(y.cols()-1)-x.col(x.cols()-1)).cwiseQuotient(scale)/1e-5;
        }
        // Dual form of the same damped least-squares system (nx rather than nu*N).
        Matrix normal=sensitivity*sensitivity.transpose()+damping*Matrix::Identity(p.x0.size(),p.x0.size());
        Vector direction=-sensitivity.transpose()*normal.ldlt().solve(residual);
        bool accepted=false;
        for(double a : {1.,.5,.25,.1,.05,.01}) {
            Matrix v=u;
            for(int j=0;j<variables;++j) v(j%u.rows(),j/u.rows())+=a*direction[j];
            for(Eigen::Index k=0;k<v.cols()-1;++k) v.col(k)=clamp(v.col(k),p.u_max);
            Matrix y=rollout(p,v,s.final_time);
            if((y.col(y.cols()-1)-p.target).cwiseQuotient(scale).norm()<residual.norm() && minimum_segment_clearance(p,y)>=-p.constraint_tolerance) {
                u=v; x=y; ++s.projection_steps; damping=std::max(1e-8,damping/3); accepted=true; break;
            }
        }
        if(!accepted) {damping*=10; if(damping>1e8) break;}
    }
    s.states=x; s.controls=u; s.controls.col(u.cols()-1)=u.col(u.cols()-2);
    s.termination_reason=(x.col(x.cols()-1)-p.target).norm()<=p.projection_tolerance?"terminal_projection_tolerance_met":"terminal_projection_stalled";
}
} // anonymous namespace

Solution solve(const Problem& p,const Options& o) {
    validate(p,o);
    const bool penalty=o.algorithm==Algorithm::penalty_free_time_ddp;
    const bool waypoint=o.algorithm==Algorithm::waypoint_pddp;
    const bool geo=geometric(o.algorithm);
    const bool defect=o.algorithm==Algorithm::terminal_defect_pddp || geo;
    const int nx=static_cast<int>(p.x0.size()),nu=static_cast<int>(p.u_max.size());
    const int n=penalty?p.intervals+1:static_cast<int>(std::floor(p.tf/p.dt));
    const int segments=waypoint?static_cast<int>(p.waypoints.cols())+1:1;
    require(n>=2*segments && n%segments==0,"Each waypoint segment requires an equal integer grid of at least two nodes");
    const double h=1.0/(n/segments-1);
    double tf=p.tf,epsilon=o.epsilon,sigma=penalty?1:1;
    Vector theta=Vector::Constant(segments,tf/segments),dtheta=Vector::Zero(segments);
    double tmin=penalty || o.algorithm==Algorithm::free_time_pddp?15:(waypoint?5:std::max(tf-20,5.));
    double tmax=penalty || o.algorithm==Algorithm::free_time_pddp?55:(waypoint?15:tf+(geo?7:20));
    if(geo && p.minimum_time>0) tmin=p.minimum_time;
    if(geo && p.maximum_time>0) tmax=p.maximum_time;
    Matrix u=Matrix::Zero(nu,n),x(nx,n),xnew(nx,n),dx=Matrix::Zero(nx,n);
    if(p.initial_controls.size()) {
        require(p.initial_controls.rows()==nu && p.initial_controls.cols()==n && p.initial_controls.allFinite(),"Initial controls must have nu x N finite entries");
        require((p.initial_controls.cwiseAbs().colwise()-p.u_max).maxCoeff()<=0,"Initial controls violate bounds");
        u=p.initial_controls;
    } else if(o.algorithm==Algorithm::free_time_pddp) {
        std::mt19937 rng(4); std::uniform_real_distribution<double> uniform(-1,1);
        for(int k=0;k<n-1;++k) for(int j=0;j<nu;++j) u(j,k)=p.u_max[j]*uniform(rng);
    }
    if(defect) {
        guidance(p,n,o.algorithm==Algorithm::geometric_safety_pddp_3d,x,u);
        tf=std::clamp(tf,tmin,tmax);
        for(int k=0;k<n-1;++k)u.col(k)=clamp(u.col(k),p.u_max);
        u.col(n-1)=u.col(n-2);x=rollout(p,u,tf);
    }
    else { x.col(0)=p.x0; for(int k=0;k<n-1;++k) x.col(k+1)=x.col(k)+(penalty?tf/(n-1):tf*h)*p.dynamics->f(x.col(k),u.col(k)); }
    xnew=x; xnew.col(0)=p.x0;
    Solution s; s.name=p.name; s.algorithm=algorithm_name(o.algorithm); s.termination_reason="maximum_iterations_reached";
    std::vector<Vector> feed(n),timegain(n),qu(n);
    std::vector<Matrix> feedback(n),quu(n),qux(n);
    Matrix lastFiniteX,lastFiniteU;double lastFiniteTime=tf;
    Matrix bestx,bestu; double besttf=tf,bestcost=infinity;
    for(int iteration=0;iteration<o.max_iterations;++iteration) {
        s.iterations=iteration+1;
        const Matrix nominalX=x,nominalU=u;const double nominalTime=tf;
        double j=p.cost->terminal(x.col(n-1),p.target);
        for(int k=0;k<n-1;++k) {
            const int seg=waypoint?std::min(k/(n/segments),segments-1):0;
            j+=(penalty?tf/(n-1):(waypoint?theta[seg]:tf)*h)*(p.cost->running(x.col(k),p.target,u.col(k))+(penalty?1:0));
        }
        if(!std::isfinite(j) || !x.allFinite() || !u.allFinite()) {
            s.termination_reason="nonfinite_trajectory";s.numerical_diagnostics.failure_identifier="nonfinite_nominal";
            if(lastFiniteX.size()){x=lastFiniteX;u=lastFiniteU;tf=lastFiniteTime;}
            break;
        }
        lastFiniteX=x;lastFiniteU=u;lastFiniteTime=tf;
        if(!defect || iteration==0)s.trial_costs.push_back(j);
        if(!defect || iteration==0)s.time_history.push_back(tf);
        if(iteration==0) s.initial_cost=j;
        if(defect && iteration==0) {
            s.line_search_weight=10+std::abs(j)/((1-o.rho)*std::max(1.,(x.col(n-1)-p.target).norm()));
            s.merit.push_back(j+s.line_search_weight*(x.col(n-1)-p.target).norm());
            s.accepted_costs.push_back(j);
        }
        if(!defect && j<bestcost) { bestcost=j; bestx=x; bestu=u; besttf=tf; s.accepted_costs.push_back(j); }
        const double tolerance=penalty || o.algorithm==Algorithm::geometric_safety_pddp_3d?1e-3:1e-5;
        if(iteration>0 && std::abs(j-s.trial_costs[iteration-1])<tolerance) { s.termination_reason="cost_change_below_tolerance"; break; }
        Vector vx=p.cost->terminal_x(x.col(n-1),p.target),vxt=Vector::Zero(nx);
        Matrix vxx=p.cost->terminal_xx(x.col(n-1),p.target);
        double vt=0,vtt=0;
        Vector vtx;
        if(penalty) {
            u.col(n-1)=u.col(n-2);
            Vector f=p.dynamics->f(x.col(n-1),u.col(n-1));
            vt=vx.dot(f)+tf/(n-1);
            vtx=vxx*f+p.dynamics->fx(x.col(n-1),u.col(n-1)).transpose()*vx;
            vtt=vtx.dot(f);
            if(!std::isfinite(vtt) || std::abs(vtt)<1e-18) {s.termination_reason="singular_time_hessian";s.numerical_diagnostics.failure_identifier="singular_time_hessian"; break;}
        }
        const Vector terminal_defect=x.col(n-1)-p.target;
        double rhoq=waypoint?(iteration==0?1e-6:1e-5):0,rhov=rhoq;
        bool backward_ok=false;
        for(int retry=0;retry<80 && !backward_ok;++retry) {
            s.numerical_diagnostics.backward_retries+=retry>0?1:0;
            vx=p.cost->terminal_x(x.col(n-1),p.target); vxx=p.cost->terminal_xx(x.col(n-1),p.target);
            if(!penalty) {vt=0; vtt=0; vxt.setZero();}
            bool restart=false;
            for(int k=n-2;k>=0;--k) {
                const int seg=waypoint?std::min((k+1)/(n/segments),segments-1):0;
                const double scale=penalty?1:(waypoint?theta[seg]:tf),step=penalty?tf/(n-1):h;
                const Vector state=x.col(k),control=u.col(k),f=p.dynamics->f(state,control);
                Matrix a=Matrix::Identity(nx,nx)+scale*step*p.dynamics->fx(state,control);
                Matrix b=scale*step*p.dynamics->fu(state,control);
                Vector c=step*f;
                Vector nextvx=vx; Matrix nexth=vxx;
                if(defect && k==n-2) nextvx+=vxx*terminal_defect;
                if(penalty && k==n-2) { nextvx-=vtx*(vt/vtt); nexth-=vtx*vtx.transpose()/vtt; }
                Vector qx=scale*step*p.cost->running_x(state,p.target,control)+a.transpose()*nextvx;
                qu[k]=scale*step*p.cost->running_u(state,p.target,control)+b.transpose()*nextvx;
                Matrix reg=nexth+rhoq*Matrix::Identity(nx,nx);
                Matrix qxx=scale*step*p.cost->running_xx(state,p.target,control)+a.transpose()*reg*a;
                quu[k]=scale*step*p.cost->running_uu(state,p.target,control)+b.transpose()*reg*b;
                Matrix qxu=scale*step*p.cost->running_xu(state,p.target,control)+a.transpose()*reg*b;
                qux[k]=qxu.transpose();
                double qt=0,qtt=0; Vector qxt=Vector::Zero(nx),qut=Vector::Zero(nu);
                if(!penalty) {
                    qt=step*p.cost->running(state,p.target,control)+vt+c.dot(vx);
                    if(defect && k==n-2) qt+=c.dot(vxx*terminal_defect)+vxt.dot(terminal_defect);
                    qxt=step*p.cost->running_x(state,p.target,control)+a.transpose()*vxt+a.transpose()*reg*c;
                    qut=step*p.cost->running_u(state,p.target,control)+b.transpose()*vxt+b.transpose()*reg*c;
                    qtt=vtt+2*c.dot(vxt)+c.dot(reg*c);
                }
                if(waypoint && seg>0 && k+1==seg*(n/segments)) {
                    Vector d=Vector::Zero(nx); d.head(2)=state.head(2)-p.waypoints.col(seg-1);
                    qx+=50*scale*step*d; qxx+=50*scale*step*Matrix::Identity(nx,nx);
                }
                for(const auto& constraint:p.constraints) {
                    const double g=scale*step*constraint.g(state,control);
                    if(g<1e-5) { Vector grad=scale*step*constraint.gx(state,control); qx+=((penalty?.004:.005)+g*sigma)*grad; qxx+=sigma*grad*grad.transpose(); }
                }
                if(!quu[k].allFinite()) {
                    s.numerical_diagnostics.failure_identifier="nonfinite_control_hessian";break;
                }
                Eigen::LLT<Matrix> chol(quu[k]);
                if(waypoint && chol.info()!=Eigen::Success) {rhoq*=2;restart=true;break;}
                Matrix rhs(nu,nx+2),gains;
                rhs.col(0)=-qu[k];rhs.middleCols(1,nx)=-qux[k];rhs.col(nx+1)=-qut;
                if(!solve_control_system(quu[k],rhs,gains,s.numerical_diagnostics))break;
                feed[k]=gains.col(0);feedback[k]=gains.middleCols(1,nx);timegain[k]=gains.col(nx+1);
                vx=qx+feedback[k].transpose()*qu[k]+qxu*feed[k]+feedback[k].transpose()*quu[k]*feed[k];
                vxx=qxx+qxu*feedback[k]+feedback[k].transpose()*qux[k]+feedback[k].transpose()*quu[k]*feedback[k];
                if(!penalty) {
                    vt=qt+timegain[k].dot(qu[k])+qut.dot(feed[k])+timegain[k].dot(quu[k]*feed[k]);
                    vxt=qxt+qxu*timegain[k]+feedback[k].transpose()*qut+feedback[k].transpose()*quu[k]*timegain[k];
                    vtt=qtt+2*qut.dot(timegain[k])+timegain[k].dot(quu[k]*timegain[k])+rhov;
                    if(!std::isfinite(vtt)) {s.numerical_diagnostics.failure_identifier="nonfinite_time_hessian";break;}
                    if(waypoint && !(vtt>0)) {rhov*=2; restart=true; break;}
                    if(waypoint && (k==0 || k+1==seg*(n/segments)))
                        dtheta[seg]=p.free_time?-(vt+vxt.dot(dx.col(k)))/vtt:0;
                }
                if(k==0) backward_ok=true;
            }
            if(!restart) break;
        }
        if(!backward_ok) {
            if(s.termination_reason=="maximum_iterations_reached")s.termination_reason="backward_pass_failed";
            if(s.numerical_diagnostics.failure_identifier.empty())s.numerical_diagnostics.failure_identifier="backward_retries_exhausted";
            break;
        }
        sigma*=penalty?1.15:2;
        double change=0;
        if(defect) {
            if(!std::isfinite(vtt) || std::abs(vtt)<1e-18 || !std::isfinite(vt)) {
                s.termination_reason="singular_time_hessian";s.numerical_diagnostics.failure_identifier="singular_time_hessian";break;
            }
            change=-vt/vtt;
            auto trial=defect_line_search(p,o,x,u,tf,tmin,tmax,change,feed,feedback,timegain,s.line_search_weight,iteration+1);
            if(iteration==0)s.merit.front()=trial.trials.front()[11];
            s.line_search_trials.insert(s.line_search_trials.end(),trial.trials.begin(),trial.trials.end());
            if(!trial.accepted){s.termination_reason="line_search_no_descent";break;}
            x=trial.states;u=trial.controls;tf=trial.time;xnew=x;
            ++s.line_search_accepted_steps;
            s.trial_costs.push_back(trial.cost);s.accepted_costs.push_back(trial.cost);s.merit.push_back(trial.merit);
            s.time_history.push_back(tf);
            continue;
        }
        if(!penalty && !waypoint) {
            if(!std::isfinite(vtt) || std::abs(vtt)<1e-18) {s.termination_reason="singular_time_hessian";s.numerical_diagnostics.failure_identifier="singular_time_hessian"; break;}
            change=-vt/vtt; tf=std::clamp(tf+change,tmin,tmax);
        }
        if(waypoint) {theta+=dtheta; for(int z=0;z<segments;++z) theta[z]=std::clamp(theta[z],tmin,tmax); tf=theta.sum();}
        for(int k=0;k<n-1;++k) {
            const int seg=waypoint?std::min(k/(n/segments),segments-1):0;
            Vector delta=xnew.col(k)-x.col(k);
            Vector candidate=u.col(k)+(penalty?o.alpha:(defect?epsilon:o.alpha))*feed[k];
            candidate+=(penalty?o.alpha:1.0)*feedback[k]*delta;
            if(!penalty) candidate+=timegain[k]*(waypoint?dtheta[seg]:change);
            if(waypoint) dx.col(k)=delta;
            if(geo) {
                const int dim=static_cast<int>(p.obstacle_centers.rows());
                for(Eigen::Index ob=0;ob<p.obstacle_centers.cols();++ob) {
                    const double dist=(xnew.col(k).head(dim)-p.obstacle_centers.col(ob)).norm();
                    const double radius=p.obstacle_radii[ob]+p.safety_margin+p.enforcement_margin;
                    const double hv=dist*dist-radius*radius;
                    if(hv<0) {
                        if(dist==0) throw std::runtime_error("Geometric correction singular at obstacle center");
                        const double adjust=(p.safety_gain*std::abs(hv)-2*dist)/(2*dist);
                        if(p.obstacle_centers.cols()==1) candidate.array()+=adjust;
                        else candidate[dim==3?1:0]+=adjust;
                    }
                }
            }
            if(!candidate.allFinite()) {
                s.numerical_diagnostics.failure_identifier="nonfinite_control_update";break;
            }
            u.col(k)=clamp(candidate,p.u_max);
            xnew.col(k+1)=xnew.col(k)+(penalty?tf/(n-1):(waypoint?theta[seg]:tf)*h)*p.dynamics->f(xnew.col(k),u.col(k));
            if(!xnew.col(k+1).allFinite()) {
                s.numerical_diagnostics.failure_identifier="nonfinite_candidate_rollout";break;
            }
        }
        if(!s.numerical_diagnostics.failure_identifier.empty()) {
            x=nominalX;u=nominalU;tf=nominalTime;s.termination_reason="nonfinite_trajectory";break;
        }
        if(penalty) {
            const double dtff=-vt/vtt,dtfb=-vtx.dot(xnew.col(n-1)-x.col(n-1))/vtt;
            tf=std::clamp(tf+.4*(dtff+dtfb),tmin,tmax);
        }
        x=xnew;
    }
    if(s.trial_costs.empty()) throw std::runtime_error("No finite nominal trajectory");
    if(penalty) {x=bestx;u=bestu;tf=besttf;}
    s.states=x; s.controls=u; s.controls.col(n-1)=u.col(n-2); s.final_time=tf;
    if(geo && s.numerical_diagnostics.failure_identifier.empty()) {
        project_terminal(p,s);
        s.projected_cost=objective(p,s.states,s.controls,s.final_time);
        refine_geometric(p,s);
    }
    s.time=Vector::LinSpaced(n,0,s.final_time);
    s.returned_cost=objective(p,s.states,s.controls,s.final_time,penalty);
    s.metrics=compute_metrics(p,s);
    return s;
}
} // namespace ddpack
