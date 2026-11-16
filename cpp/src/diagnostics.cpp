// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#include <ddpack/ddpack.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace ddpack {
namespace {
double trig_integral(double angle,double rate,double h,bool sine) {
    const double z=rate*h/2;
    const double factor=std::abs(z)<1e-4?1-z*z/6+z*z*z*z/120:std::sin(z)/z;
    return h*factor*(sine?std::sin(angle+z):std::cos(angle+z));
}
void analytic_audit(const Problem& p,const Solution& s,Metrics& m) {
    const auto* planar=dynamic_cast<const PlanarDynamics*>(p.dynamics.get());
    const bool flight=dynamic_cast<const FlightDynamics*>(p.dynamics.get())!=nullptr;
    if((!planar && !flight) || !p.obstacle_centers.size()) return;
    const int dim=planar?2:3;
    const double speed=planar?planar->speed:p.x0[3];
    Vector x=p.x0; double minimum=infinity,maximum_step=0;
    for(Eigen::Index k=0;k<s.time.size()-1;++k) {
        const double step=(s.time[k+1]-s.time[k])/64.; maximum_step=std::max(maximum_step,step);
        Vector y=x; const Vector u=s.controls.col(k);
        for(int j=0;j<=64;++j) {
            const double h=j*step; y=x;
            if(planar) {
                y[0]+=speed*trig_integral(x[2],u[0],h,false);
                y[1]+=speed*trig_integral(x[2],u[0],h,true); y[2]+=h*u[0];
            } else {
                const double g=x[4],a=u[1],b=u[0],heading=x[5];
                y[0]+=speed/2*(trig_integral(g-heading,a-b,h,false)+trig_integral(g+heading,a+b,h,false));
                y[1]+=speed/2*(trig_integral(heading+g,b+a,h,true)+trig_integral(heading-g,b-a,h,true));
                y[2]+=speed*trig_integral(g,a,h,true); y[4]+=h*a; y[5]+=h*b;
            }
            for(Eigen::Index ob=0;ob<p.obstacle_centers.cols();++ob)
                minimum=std::min(minimum,(y.head(dim)-p.obstacle_centers.col(ob)).norm()-p.obstacle_radii[ob]-p.safety_margin);
        }
        x=y;
    }
    m.analytic_zoh_clearance_lower_bound=minimum-std::abs(speed)*maximum_step/2
        -1e-10*(1+p.x0.norm()+std::abs(speed)*(s.time[s.time.size()-1]-s.time[0]));
    m.analytic_zoh_terminal_error=(x-p.target).norm();
}
}
double minimum_segment_clearance(const Problem& p,const Matrix& x) {
    double clearance=infinity;
    const auto dim=p.obstacle_centers.rows();
    for(Eigen::Index ob=0;ob<p.obstacle_centers.cols();++ob) {
        for(Eigen::Index k=0;k<x.cols()-1;++k) {
            Vector start=x.col(k).head(dim),delta=x.col(k+1).head(dim)-start;
            double fraction=delta.squaredNorm()>0?std::clamp((p.obstacle_centers.col(ob)-start).dot(delta)/delta.squaredNorm(),0.,1.):0;
            clearance=std::min(clearance,(start+fraction*delta-p.obstacle_centers.col(ob)).norm()-p.obstacle_radii[ob]-p.safety_margin);
        }
    }
    return clearance;
}
Metrics compute_metrics(const Problem& p,const Solution& s) {
    Metrics m;
    if(s.states.rows()!=p.x0.size() || s.states.cols()<2 || s.controls.rows()!=p.u_max.size() || s.controls.cols()!=s.states.cols() || s.time.size()!=s.states.cols()) return m;
    m.finite=s.states.allFinite() && s.controls.allFinite() && s.time.allFinite() && std::isfinite(s.returned_cost);
    if(!m.finite) return m;
    const double duration=s.time[s.time.size()-1]-s.time[0];
    m.time_bound_violation=std::max(0.,std::max(p.minimum_time>0?p.minimum_time-duration:0.,
        p.maximum_time>0?duration-p.maximum_time:0.));
    m.time_bounds_pass=m.time_bound_violation==0;
    m.terminal_error=p.terminal_error?p.terminal_error(s.states.col(s.states.cols()-1),p.target):(s.states.col(s.states.cols()-1)-p.target).norm();
    m.control_violation=0; m.constraint_violation=0; m.euler_defect=0; m.rk4_defect=0;
    for(Eigen::Index k=0;k<s.states.cols();++k) {
        m.control_violation=std::max(m.control_violation,(s.controls.col(k).cwiseAbs()-p.u_max).maxCoeff());
        for(const auto& c:p.constraints) {
            const double g=c.g(s.states.col(k),s.controls.col(k));
            if(!std::isfinite(g)) {m.finite=false; return m;}
            m.constraint_violation=std::max(m.constraint_violation,-g);
        }
        if(k==s.states.cols()-1) break;
        const double dt=s.time[k+1]-s.time[k];
        if(!(dt>0)) {m.finite=false; return m;}
        Vector x=s.states.col(k),u=s.controls.col(k),f1=p.dynamics->f(x,u),prediction=x;
        const double substep=dt/4;
        for(int sub=0;sub<4;++sub) {
            Vector a=p.dynamics->f(prediction,u),b=p.dynamics->f(prediction+.5*substep*a,u);
            Vector c=p.dynamics->f(prediction+.5*substep*b,u),d=p.dynamics->f(prediction+substep*c,u);
            prediction+=substep/6*(a+2*b+2*c+d);
        }
        if(!f1.allFinite() || !prediction.allFinite()) {m.finite=false; return m;}
        m.euler_defect=std::max(m.euler_defect,(s.states.col(k+1)-x-dt*f1).cwiseAbs().maxCoeff());
        m.rk4_defect=std::max(m.rk4_defect,(s.states.col(k+1)-prediction).cwiseAbs().maxCoeff());
    }
    m.minimum_clearance=minimum_segment_clearance(p,s.states);
    analytic_audit(p,s,m);
    m.numerical_checks_pass=m.finite && m.time_bounds_pass && std::isfinite(m.terminal_error) && m.terminal_error<=p.terminal_tolerance &&
        m.control_violation<=p.constraint_tolerance && m.constraint_violation<=p.constraint_tolerance &&
        m.minimum_clearance>=-p.constraint_tolerance && m.euler_defect<=p.defect_tolerance && m.rk4_defect<=p.rk4_tolerance &&
        (std::isnan(m.analytic_zoh_clearance_lower_bound) || (m.analytic_zoh_clearance_lower_bound>=0 && m.analytic_zoh_terminal_error<=p.terminal_tolerance)) &&
        (s.states.col(0)-p.x0).norm()<=1e-10 && (!p.require_cost_decrease || s.returned_cost<=s.initial_cost*(1+1e-10));
    m.solver_convergence_pass=s.termination_reason=="cost_change_below_tolerance" ||
        s.termination_reason=="terminal_projection_tolerance_met" || s.termination_reason=="geometric_refinement_feasible";
    const bool numerical_failure=!s.numerical_diagnostics.failure_identifier.empty();
    m.success=m.numerical_checks_pass && m.solver_convergence_pass && !numerical_failure;
    m.status=numerical_failure?"numerical_failure":(m.success?"converged":
        (m.numerical_checks_pass?"feasible_not_converged":"failed_checks"));
    return m;
}
namespace {
std::ofstream output(const std::filesystem::path& p) {
    std::ofstream stream(p); stream.exceptions(std::ios::badbit|std::ios::failbit); stream<<std::setprecision(17); return stream;
}
std::string escaped(const std::string& s) {
    std::string v; for(unsigned char c:s) {if(c=='"' || c=='\\') v+='\\'; if(c>=32) v+=char(c); } return v;
}
void number(std::ostream& f,double v) { if(std::isfinite(v)) f<<v; else f<<"null"; }
void matrix_csv(const std::filesystem::path& p,const Matrix& m) {
    auto f=output(p); for(Eigen::Index i=0;i<m.rows();++i) {for(Eigen::Index j=0;j<m.cols();++j) {if(j)f<<','; f<<m(i,j);} f<<'\n';}
}
}
void write_artifacts(const Problem& p,const Options& o,const Solution& s,const std::string& directory) {
    namespace fs=std::filesystem;
    fs::path path=fs::u8path(directory);
    if(fs::exists(path)) throw std::runtime_error("Output directory already exists: "+directory);
    fs::create_directories(path);
    matrix_csv(path/"states.csv",s.states); matrix_csv(path/"controls.csv",s.controls); matrix_csv(path/"time.csv",s.time.transpose());
    if(s.refinement_starts.size()) {
        matrix_csv(path/"refinement_starts.csv",s.refinement_starts);
        Matrix h(s.refinement_penalty_history.size(),1);
        for(Eigen::Index i=0;i<h.rows();++i)h(i,0)=s.refinement_penalty_history[i];
        matrix_csv(path/"refinement_penalty_history.csv",h);
    }
    auto history=output(path/"cost_history.csv"); history<<"iteration,trial,best_so_far\n"; double best=infinity;
    for(std::size_t i=0;i<s.trial_costs.size();++i) {best=std::min(best,s.trial_costs[i]); history<<i+1<<','<<s.trial_costs[i]<<','<<best<<'\n';}
    if(!s.line_search_trials.empty()) {
        auto trials=output(path/"line_search_trials.csv");
        trials<<"iteration,attempt,fraction,epsilon,time_before,time_after,weight,cost_before,cost_after,defect_before,defect_after,merit_before,merit_after,geometry_before,geometry_after,geometry_weight,finite,accepted\n";
        for(const auto& row:s.line_search_trials){for(std::size_t k=0;k<row.size();++k){if(k)trials<<',';trials<<row[k];}trials<<'\n';}
    }
    auto metrics=output(path/"metrics.json"); metrics<<"{\n  \"success\": "<<(s.metrics.success?"true":"false");
    metrics<<",\n  \"status\": \""<<escaped(s.metrics.status)<<"\",\n  \"termination_reason\": \""<<escaped(s.termination_reason)<<"\",\n  \"numerical_failure_identifier\": \""<<escaped(s.numerical_diagnostics.failure_identifier)<<"\"";
    metrics<<",\n  \"numerical_checks_pass\": "<<(s.metrics.numerical_checks_pass?"true":"false")
           <<",\n  \"solver_convergence_pass\": "<<(s.metrics.solver_convergence_pass?"true":"false")
           <<",\n  \"time_bounds_pass\": "<<(s.metrics.time_bounds_pass?"true":"false");
    const std::vector<std::pair<std::string,double>> fields={{"linear_solves",double(s.numerical_diagnostics.linear_solves)},{"regularized_solves",double(s.numerical_diagnostics.regularized_solves)},{"maximum_damping",s.numerical_diagnostics.maximum_damping},{"backward_retries",double(s.numerical_diagnostics.backward_retries)},{"terminal_error",s.metrics.terminal_error},{"control_violation",s.metrics.control_violation},
        {"analytic_zoh_clearance_lower_bound",s.metrics.analytic_zoh_clearance_lower_bound},{"analytic_zoh_terminal_error",s.metrics.analytic_zoh_terminal_error},
        {"constraint_violation",s.metrics.constraint_violation},{"time_bound_violation",s.metrics.time_bound_violation},{"euler_defect",s.metrics.euler_defect},{"rk4_defect",s.metrics.rk4_defect},
        {"minimum_clearance",s.metrics.minimum_clearance},{"returned_cost",s.returned_cost},{"initial_cost",s.initial_cost},{"final_time",s.final_time},
        {"refinement_baseline_cost",s.refinement_baseline_cost},{"refinement_baseline_rate_cost",s.refinement_baseline_rate_cost},
        {"refinement_rate_cost",s.refinement_rate_cost},{"refinement_selected_start",double(s.refinement_selected_start)}};
    for(const auto& f:fields) {metrics<<",\n  \""<<f.first<<"\": "; number(metrics,f.second);} metrics<<"\n}\n";
    auto config=output(path/"config.json"); config<<"{\"problem\":\""<<escaped(p.name)<<"\",\"algorithm\":\""<<algorithm_name(o.algorithm)
        <<"\",\"max_iterations\":"<<o.max_iterations<<",\"alpha\":"<<o.alpha<<",\"epsilon\":"<<o.epsilon<<",\"rho\":"<<o.rho<<",\"update_epsilon\":"<<o.update_epsilon<<",\"tf\":"<<p.tf<<",\"dt\":"<<p.dt
        <<",\"geometric_refinement_iterations\":"<<p.geometric_refinement_iterations<<",\"geometric_refinement_knots\":"<<p.geometric_refinement_knots
        <<",\"geometric_control_rate_weight\":"<<p.geometric_control_rate_weight<<",\"geometric_refinement_free_time\":"<<(p.geometric_refinement_free_time?"true":"false")
        <<",\"minimum_time\":"<<p.minimum_time<<",\"maximum_time\":"<<p.maximum_time<<"}\n";
    auto environment=output(path/"environment.txt"); environment<<"DDPACK native C++17\nEigen ";
#ifdef EIGEN_VERSION_STRING
    environment<<EIGEN_VERSION_STRING<<'\n';
#else
    environment<<EIGEN_WORLD_VERSION<<'.'<<EIGEN_MAJOR_VERSION<<'.'<<EIGEN_MINOR_VERSION<<'\n';
#endif
#ifdef _MSC_VER
    environment<<"MSVC "<<_MSC_VER<<'\n';
#elif defined(__VERSION__)
    environment<<__VERSION__<<'\n';
#endif
    auto summary=output(path/"summary.txt"); summary<<s.name<<'\n'<<s.algorithm<<"\ntermination: "<<s.termination_reason<<"\niterations: "<<s.iterations<<"\nprojection steps: "<<s.projection_steps<<"\nacceptance: "<<(s.metrics.success?"PASS":"FAIL")<<'\n';
}
} // namespace ddpack
