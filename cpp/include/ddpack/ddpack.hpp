// Copyright (c) 2026 Feiran Xia
// Beijing Institute of Technology, China
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <limits>
static_assert(EIGEN_VERSION_AT_LEAST(3,4,0), "DDPACK requires Eigen >= 3.4");

namespace ddpack {
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
constexpr double infinity = std::numeric_limits<double>::infinity();
// Physical-time dynamics. The solver applies time scaling exactly once.
struct Dynamics {
    virtual ~Dynamics() = default;
    virtual Vector f(const Vector& x, const Vector& u) const = 0;
    virtual Matrix fx(const Vector& x, const Vector& u) const = 0;
    virtual Matrix fu(const Vector& x, const Vector& u) const = 0;
};
struct LinearDynamics final : Dynamics {
    Matrix A, B;
    Vector drift;
    LinearDynamics(Matrix a, Matrix b, Vector d);
    Vector f(const Vector&, const Vector&) const override;
    Matrix fx(const Vector&, const Vector&) const override;
    Matrix fu(const Vector&, const Vector&) const override;
};
struct PendulumDynamics final : Dynamics {
    double mass = 1, length = 1, gravity = 9.81, damping = .1;
    Vector f(const Vector&, const Vector&) const override;
    Matrix fx(const Vector&, const Vector&) const override;
    Matrix fu(const Vector&, const Vector&) const override;
};
struct PlanarDynamics final : Dynamics {
    double speed = 1;
    Vector f(const Vector&, const Vector&) const override;
    Matrix fx(const Vector&, const Vector&) const override;
    Matrix fu(const Vector&, const Vector&) const override;
};
struct FlightDynamics final : Dynamics {
    Vector f(const Vector&, const Vector&) const override;
    Matrix fx(const Vector&, const Vector&) const override;
    Matrix fu(const Vector&, const Vector&) const override;
};
struct ThermalDynamics final : Dynamics {
    Vector f(const Vector&, const Vector&) const override;
    Matrix fx(const Vector&, const Vector&) const override;
    Matrix fu(const Vector&, const Vector&) const override;
};
// Continuous running-cost density and terminal cost, with analytic derivatives.
struct Cost {
    virtual ~Cost() = default;
    virtual double terminal(const Vector&, const Vector&) const = 0;
    virtual Vector terminal_x(const Vector&, const Vector&) const = 0;
    virtual Matrix terminal_xx(const Vector&, const Vector&) const = 0;
    virtual double running(const Vector&, const Vector&, const Vector&) const = 0;
    virtual Vector running_x(const Vector&, const Vector&, const Vector&) const = 0;
    virtual Vector running_u(const Vector&, const Vector&, const Vector&) const = 0;
    virtual Matrix running_xx(const Vector&, const Vector&, const Vector&) const = 0;
    virtual Matrix running_uu(const Vector&, const Vector&, const Vector&) const = 0;
    virtual Matrix running_xu(const Vector&, const Vector&, const Vector&) const = 0;
};
struct QuadraticCost final : Cost {
    Matrix Qf, Q, R;
    QuadraticCost(Matrix terminal_weight, Matrix state_weight, Matrix control_weight);
    double terminal(const Vector&, const Vector&) const override;
    Vector terminal_x(const Vector&, const Vector&) const override;
    Matrix terminal_xx(const Vector&, const Vector&) const override;
    double running(const Vector&, const Vector&, const Vector&) const override;
    Vector running_x(const Vector&, const Vector&, const Vector&) const override;
    Vector running_u(const Vector&, const Vector&, const Vector&) const override;
    Matrix running_xx(const Vector&, const Vector&, const Vector&) const override;
    Matrix running_uu(const Vector&, const Vector&, const Vector&) const override;
    Matrix running_xu(const Vector&, const Vector&, const Vector&) const override;
};
struct Constraint {
    // Scalar feasibility convention: g(x,u) >= 0.
    std::function<double(const Vector&, const Vector&)> g;
    std::function<Vector(const Vector&, const Vector&)> gx;
};
Constraint state_lower_bound(int index, double lower, int state_dimension);
enum class Algorithm { free_time_pddp, waypoint_pddp, terminal_defect_pddp,
    geometric_safety_pddp_2d, geometric_safety_pddp_3d, penalty_free_time_ddp };
std::string algorithm_name(Algorithm);
struct Options {
    Algorithm algorithm = Algorithm::free_time_pddp;
    int max_iterations = 80;
    double alpha = .2, epsilon = .2, rho = .05, update_epsilon = .95;
};
struct Problem {
    std::string name = "unnamed";
    Vector x0, target, u_max;
    double tf = 15, dt = .3;
    int intervals = 50;
    std::shared_ptr<const Dynamics> dynamics;
    std::shared_ptr<const Cost> cost;
    std::vector<Constraint> constraints;
    Matrix waypoints; // Position coordinates in columns, as in MATLAB.
    bool free_time = false;
    Matrix initial_controls; // Optional deterministic controls; columns are nodes.
    Matrix obstacle_centers;
    Vector obstacle_radii;
    double safety_margin = 0, enforcement_margin = 0, safety_gain = .5;
    double minimum_time = 0, maximum_time = 0; // Geometric algorithms only; zero means default.
    double projection_tolerance = 0;
    int projection_max_iterations = 20;
    int geometric_refinement_iterations = 0, geometric_refinement_knots = 12;
    double geometric_control_rate_weight = .02;
    bool geometric_refinement_free_time = false;
    double terminal_tolerance = .1, constraint_tolerance = 1e-6;
    double defect_tolerance = 1e-6, rk4_tolerance = infinity;
    bool require_cost_decrease = true;
    std::function<double(const Vector&, const Vector&)> terminal_error;
};
struct Metrics {
    double analytic_zoh_clearance_lower_bound = std::numeric_limits<double>::quiet_NaN();
    double analytic_zoh_terminal_error = std::numeric_limits<double>::quiet_NaN();
    double terminal_error = infinity, control_violation = infinity;
    double constraint_violation = infinity, euler_defect = infinity, rk4_defect = infinity;
    double minimum_clearance = infinity;
    double time_bound_violation = infinity;
    bool time_bounds_pass = false;
    bool finite = false, success = false;
    bool numerical_checks_pass = false, solver_convergence_pass = false;
    std::string status = "failed_checks";
};
struct NumericalDiagnostics {
    int linear_solves = 0, regularized_solves = 0, backward_retries = 0;
    double maximum_damping = 0;
    std::string failure_identifier;
};
struct Solution {
    std::string name, algorithm, termination_reason;
    Matrix states, controls; // States: nx x N; controls: nu x N, terminal held.
    Vector time;
    double final_time = 0, returned_cost = infinity, initial_cost = infinity;
    std::vector<double> trial_costs, accepted_costs, merit, time_history;
    std::vector<std::vector<double>> line_search_trials;
    double line_search_weight = 0;
    int line_search_accepted_steps = 0;
    int iterations = 0, projection_steps = 0;
    Matrix refinement_starts;
    std::vector<double> refinement_penalty_history;
    int refinement_selected_start = 0;
    double projected_cost = infinity, refinement_baseline_cost = infinity;
    double refinement_baseline_rate_cost = 0, refinement_rate_cost = 0;
    Metrics metrics;
    NumericalDiagnostics numerical_diagnostics;
};
Solution solve(const Problem&, const Options& = {});
Metrics compute_metrics(const Problem&, const Solution&);
double minimum_segment_clearance(const Problem&, const Matrix&);
void write_artifacts(const Problem&, const Options&, const Solution&, const std::string& directory);
} // namespace ddpack
