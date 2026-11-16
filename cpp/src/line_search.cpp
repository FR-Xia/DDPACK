// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include "line_search.hpp"
#include <algorithm>
#include <cmath>

namespace ddpack {
namespace {
double cost_of(const Problem& p,const Matrix& x,const Matrix& u,double tf) {
    double cost=p.cost->terminal(x.col(x.cols()-1),p.target);
    const double h=tf/double(x.cols()-1);
    for(Eigen::Index k=0;k<x.cols()-1;++k)cost+=h*p.cost->running(x.col(k),p.target,u.col(k));
    return cost;
}
}
DefectLineSearchResult defect_line_search(const Problem& p,const Options& o,const Matrix& x,const Matrix& u,
    double tf,double tmin,double tmax,double time_change,const std::vector<Vector>& feed,
    const std::vector<Matrix>& feedback,const std::vector<Vector>& timegain,double weight,int iteration) {
    DefectLineSearchResult result;result.states=x;result.controls=u;result.time=tf;
    result.cost=cost_of(p,x,u,tf);
    const double baseDefect=(x.col(x.cols()-1)-p.target).norm();
    const bool geo=o.algorithm==Algorithm::geometric_safety_pddp_2d || o.algorithm==Algorithm::geometric_safety_pddp_3d;
    const double baseGeometry=geo?std::max(0.,p.enforcement_margin-minimum_segment_clearance(p,x)):0;
    const double geometryWeight=geo?10*weight:0;
    result.merit=result.cost+weight*baseDefect+geometryWeight*baseGeometry;
    const double timeDirection=std::clamp(tf+time_change,tmin,tmax)-tf;
    double shrink=std::min(.5,o.update_epsilon);
    if(!std::isfinite(shrink) || shrink<=0 || shrink>=1)shrink=.5;
    const int n=static_cast<int>(x.cols());const double h=1./double(n-1);
    double step=1;
    for(int attempt=1;attempt<=24;++attempt,step*=shrink) {
        Matrix trialX=Matrix::Zero(x.rows(),n),trialU=u;trialX.col(0)=x.col(0);
        const double trialTime=std::clamp(tf+step*timeDirection,tmin,tmax);
        bool finite=std::isfinite(time_change) && std::isfinite(trialTime) && trialTime>0;
        for(int k=0;k<n-1 && finite;++k) {
            Vector control=u.col(k)+step*o.epsilon*feed[k];
            control+=feedback[k]*(trialX.col(k)-x.col(k));
            control+=timegain[k]*(trialTime-tf);
            if(geo) {
                const int dim=static_cast<int>(p.obstacle_centers.rows());
                for(Eigen::Index ob=0;ob<p.obstacle_centers.cols();++ob) {
                    const double distance=(trialX.col(k).head(dim)-p.obstacle_centers.col(ob)).norm();
                    const double radius=p.obstacle_radii[ob]+p.safety_margin+p.enforcement_margin;
                    const double clearance=distance*distance-radius*radius;
                    if(clearance<0) {
                        if(distance==0){finite=false;break;}
                        const double adjust=(p.safety_gain*std::abs(clearance)-2*distance)/(2*distance);
                        if(p.obstacle_centers.cols()==1)control.array()+=adjust;
                        else control[dim==3?1:0]+=adjust;
                    }
                }
            }
            if(!finite)break;
            trialU.col(k)=control.cwiseMax(-p.u_max).cwiseMin(p.u_max);
            trialX.col(k+1)=trialX.col(k)+trialTime*h*p.dynamics->f(trialX.col(k),trialU.col(k));
            finite=trialU.col(k).allFinite() && trialX.col(k+1).allFinite();
        }
        double trialCost=infinity,trialDefect=infinity,trialGeometry=infinity,trialMerit=infinity;
        if(finite) {
            trialU.col(n-1)=trialU.col(n-2);
            trialCost=cost_of(p,trialX,trialU,trialTime);
            trialDefect=(trialX.col(n-1)-p.target).norm();
            trialGeometry=geo?std::max(0.,p.enforcement_margin-minimum_segment_clearance(p,trialX)):0;
            trialMerit=trialCost+weight*trialDefect+geometryWeight*trialGeometry;
            finite=std::isfinite(trialMerit);
        }
        const bool accepted=finite && trialMerit<result.merit-1e-12*std::max(1.,std::abs(result.merit));
        result.trials.push_back({double(iteration),double(attempt),step,o.epsilon,tf,trialTime,weight,
            result.cost,trialCost,baseDefect,trialDefect,result.merit,trialMerit,
            baseGeometry,trialGeometry,geometryWeight,finite?1.:0.,accepted?1.:0.});
        if(accepted){result.accepted=true;result.states=trialX;result.controls=trialU;
            result.time=trialTime;result.cost=trialCost;result.merit=trialMerit;break;}
    }
    return result;
}
}
