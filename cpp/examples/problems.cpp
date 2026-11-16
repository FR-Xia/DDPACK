#include "problems.hpp"
#include <cmath>
#include <stdexcept>
namespace ddpack::examples { namespace {
Vector v(std::initializer_list<double> values) { Vector x(values.size()); int i=0; for(double a:values) x[i++]=a; return x; }
Matrix diag(std::initializer_list<double> values) {return v(values).asDiagonal();}
void cost(Problem& p,Matrix f,Matrix q,Matrix r) {p.cost=std::make_shared<QuadraticCost>(f,q,r);}
void linear(Problem& p,Matrix a,Matrix b,Vector d) {p.dynamics=std::make_shared<LinearDynamics>(a,b,d);}
}
std::vector<std::string> names() {return {"vibration","planar","terminal_defect","obstacle_2d","obstacle_3d","reactor","pendulum","thermal","storage"};}
Case make(const std::string& name) {
    Case c; auto& p=c.problem; auto& o=c.options; p.name=name; p.constraint_tolerance=1e-8;
    if(name=="vibration") {
        p.x0=v({1.2,0}); p.target=v({0,0}); p.u_max=v({2});
        Matrix a(2,2); a<<0,1,-1,-.4; Matrix b(2,1); b<<0,.2;
        linear(p,a,b,v({0,0})); cost(p,diag({200,60}),diag({2,.5}),diag({.5})); p.terminal_tolerance=.01;
    } else if(name=="planar") {
        o.algorithm=Algorithm::waypoint_pddp; o.max_iterations=100; o.alpha=.4;
        p.x0=v({0,0}); p.target=v({5,2}); p.u_max=v({2}); p.tf=5; p.dt=.05;
        Matrix b(2,1); b<<0,1; linear(p,Matrix::Zero(2,2),b,v({1,0}));
        cost(p,diag({500,500}),diag({1,1}),diag({.2})); p.terminal_tolerance=.05; p.rk4_tolerance=.01;
    } else if(name=="terminal_defect" || name=="obstacle_2d") {
        const bool geo=name=="obstacle_2d";
        o.algorithm=geo?Algorithm::geometric_safety_pddp_2d:Algorithm::terminal_defect_pddp;
        p.x0=v({0,0,0}); p.target=v({5,geo?0:1.2,0}); p.tf=10; p.dt=.2; p.u_max=v({1.5});
        if(geo) p.dynamics=std::make_shared<PlanarDynamics>();
        else {Matrix b(3,1); b<<0,1,0; linear(p,Matrix::Zero(3,3),b,v({1,0,0}));}
        cost(p,diag({200,200,40}),diag({1,geo?1.:2.,.2}),diag({.2}));
        p.terminal_tolerance=geo?.005:.001; p.rk4_tolerance=geo?.02:.01;
        if(geo) {
            p.obstacle_centers=v({2.5,0}); p.obstacle_radii=v({.22}); p.safety_margin=.03; p.enforcement_margin=.12;
            p.minimum_time=5.0; p.maximum_time=6.0; p.projection_tolerance=.005; p.projection_max_iterations=30; p.geometric_refinement_iterations=80; p.geometric_refinement_free_time=true;
        }
    } else if(name=="obstacle_3d") {
        o.algorithm=Algorithm::geometric_safety_pddp_3d;
        const double speed=std::sqrt(30.)/5,heading=std::atan2(2.,5.),gamma=std::atan2(1.,std::sqrt(29.));
        p.x0=v({0,0,0,speed,gamma,heading}); p.target=v({5,2,1,speed,gamma,heading}); p.tf=5; p.dt=.1; p.u_max=v({1.5,1.5});
        p.dynamics=std::make_shared<FlightDynamics>(); cost(p,diag({1000,1000,1000,20,100,100}),diag({.05,.05,.05,.01,.05,.05}),diag({.2,.2}));
        p.obstacle_centers=v({5*24./49,2*24./49,24./49}); p.obstacle_radii=v({.18}); p.safety_margin=.04; p.enforcement_margin=.15; p.safety_gain=.2;
        p.minimum_time=5.0; p.maximum_time=6.0; p.projection_tolerance=.005; p.projection_max_iterations=30; p.geometric_refinement_iterations=80; p.geometric_refinement_free_time=true; p.terminal_tolerance=.005; p.rk4_tolerance=.02;
    } else if(name=="reactor" || name=="thermal" || name=="storage") {
        o.algorithm=Algorithm::penalty_free_time_ddp;
        if(name=="reactor") {
            p.x0=v({4}); p.target=v({0}); p.u_max=v({1}); linear(p,diag({-.3}),diag({.02}),v({0}));
            cost(p,diag({200}),diag({2}),diag({2})); o.alpha=.25; p.terminal_tolerance=.04;
        } else if(name=="thermal") {
            p.x0=v({2}); p.target=v({.032535}); p.u_max=v({1.5}); p.dynamics=std::make_shared<ThermalDynamics>();
            cost(p,diag({120}),diag({2.5}),diag({.25})); o.max_iterations=220; p.terminal_tolerance=5e-5;
            p.constraint_tolerance=1e-10;
        } else {
            p.x0=v({.65}); p.target=v({.353}); p.u_max=v({.8}); linear(p,diag({-.03}),diag({.08}),v({-.005}));
            cost(p,diag({160}),diag({3}),diag({.3})); o.max_iterations=100; p.terminal_tolerance=5e-5;
            p.constraints.push_back(state_lower_bound(0,.25,1));
        }
        p.rk4_tolerance=name=="thermal"?.05:.02;
    } else if(name=="pendulum") {
        o.algorithm=Algorithm::waypoint_pddp; o.max_iterations=120; o.alpha=.4;
        p.x0=v({std::acos(-1.),0}); p.target=v({0,0}); p.tf=5; p.dt=.05; p.u_max=v({10});
        p.dynamics=std::make_shared<PendulumDynamics>(); cost(p,diag({100,10}),diag({1,.1}),diag({.1})); p.terminal_tolerance=.01;
        p.terminal_error=[](const Vector& x,const Vector& t) { Vector d=x-t; d[0]=std::remainder(d[0],2*std::acos(-1.)); return d.norm(); };
    } else throw std::invalid_argument("Unknown example: "+name);
    return c;
}
}
