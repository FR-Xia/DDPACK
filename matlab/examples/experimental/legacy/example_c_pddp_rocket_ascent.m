%% Rocket ascent optimal control using existing C-PDDP
% State:   x = [altitude; vertical_velocity]
% Control: u = commanded upward acceleration
% Goal: climb to a target altitude and finish with near-zero vertical speed.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== Rocket ascent optimal-control example (C-PDDP) ===\n');

g = 9.81;
target_altitude = 120.0;

A = [0.0, 1.0;
     0.0, 0.0];
B = [0.0;
     1.0];
drift = [0.0; -g];
dyn = PointMassDynamics(A, B, drift);

x0 = [0.0; 0.0];
xf = [target_altitude; 0.0];
tf = 20.0;
dt = 0.5;
u_max = 25.0;
num_iter = 80;
alpha = 0.35;

Q_f = diag([200.0, 80.0]);
Q_s = diag([0.01, 0.05]);
R = 0.02;
cost = QuadraticCost(Q_f, Q_s, R);

solver_log = evalc('[sol, sol_init] = c_pddp(0, 0, x0, [], xf, tf, dt, dyn, cost, u_max, num_iter, alpha);'); %#ok<NASGU,ASGLU>

[t, x, u, cost_history] = collect_solution(sol, tf);
altitude = x(1, :);
velocity = x(2, :);
thrust_accel = u(1, :);

fprintf('Algorithm: C-PDDP\n');
fprintf('Final altitude error: %.4f m\n', altitude(end) - target_altitude);
fprintf('Final velocity: %.4f m/s\n', velocity(end));
fprintf('Peak commanded acceleration: %.4f m/s^2\n', max(abs(thrust_accel)));

assert(sol.error == 0, 'C-PDDP reported a numerical error.');
assert(abs(altitude(end) - target_altitude) < 1.0, 'Rocket ascent final altitude error is too large.');
assert(abs(velocity(end)) < 1.0, 'Rocket ascent final velocity is too large.');

figure('Name', 'C-PDDP Rocket Ascent', 'Position', [120, 120, 1200, 720]);

subplot(2, 2, 1);
plot(t, altitude, 'b-', 'LineWidth', 2); hold on;
yline(target_altitude, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude'); grid on;
legend('Optimized', 'Target', 'Location', 'best');

subplot(2, 2, 2);
plot(t, velocity, 'm-', 'LineWidth', 2); hold on;
yline(0.0, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vertical velocity (m/s)');
title('Vertical Velocity'); grid on;

subplot(2, 2, 3);
stairs(t, thrust_accel, 'k-', 'LineWidth', 2); hold on;
yline(u_max, 'r--'); yline(-u_max, 'r--');
xlabel('Time (s)'); ylabel('Acceleration command (m/s^2)');
title('Control Input'); grid on;

subplot(2, 2, 4);
valid_cost = cost_history(cost_history ~= 0);
plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
xlabel('Iteration'); ylabel('Cost');
title('C-PDDP Cost History'); grid on;
set(gca, 'YScale', 'log');

sgtitle('Rocket Ascent Optimal Control with C-PDDP');

function [t, x_mat, u_mat, J] = collect_solution(sol, tf)
    n = numel(sol.x{1});
    m = numel(sol.u{1});
    N = numel(sol.x);
    x_mat = zeros(n, N);
    u_mat = zeros(m, N);
    for k = 1:N
        x_mat(:, k) = sol.x{k};
        u_mat(:, k) = sol.u{k};
    end
    t = linspace(0.0, tf, N);
    J = sol.J;
end
