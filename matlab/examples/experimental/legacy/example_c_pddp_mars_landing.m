%% Mars powered descent optimal control using existing C-PDDP
% State:   x = [horizontal_position; altitude; horizontal_velocity; vertical_velocity]
% Control: u = [horizontal_acceleration; vertical_acceleration]
% Goal: land near the target site with low velocity under Mars gravity.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== Mars landing optimal-control example (C-PDDP) ===\n');

g_mars = 3.711;

A = [0.0, 0.0, 1.0, 0.0;
     0.0, 0.0, 0.0, 1.0;
     0.0, 0.0, 0.0, 0.0;
     0.0, 0.0, 0.0, 0.0];
B = [0.0, 0.0;
     0.0, 0.0;
     1.0, 0.0;
     0.0, 1.0];
drift = [0.0; 0.0; 0.0; -g_mars];
dyn = PointMassDynamics(A, B, drift);

x0 = [160.0; 140.0; -8.0; -16.0];
xf = [0.0; 0.0; 0.0; 0.0];
tf = 25.0;
dt = 0.5;
u_max = [10.0; 14.0];
num_iter = 100;
alpha = 0.35;

Q_f = diag([200.0, 200.0, 100.0, 120.0]);
Q_s = diag([0.02, 0.02, 0.04, 0.04]);
R = diag([0.04, 0.04]);
cost = QuadraticCost(Q_f, Q_s, R);

solver_log = evalc('[sol, sol_init] = c_pddp(0, 0, x0, [], xf, tf, dt, dyn, cost, u_max, num_iter, alpha);'); %#ok<NASGU,ASGLU>

[t, x, u, cost_history] = collect_solution(sol, tf);
horizontal_position = x(1, :);
altitude = x(2, :);
horizontal_velocity = x(3, :);
vertical_velocity = x(4, :);

fprintf('Algorithm: C-PDDP\n');
fprintf('Final position: x = %.4f m, z = %.4f m\n', horizontal_position(end), altitude(end));
fprintf('Final velocity: vx = %.4f m/s, vz = %.4f m/s\n', horizontal_velocity(end), vertical_velocity(end));
fprintf('Peak acceleration command: ax = %.4f, az = %.4f m/s^2\n', max(abs(u(1, :))), max(abs(u(2, :))));

assert(sol.error == 0, 'C-PDDP reported a numerical error.');
assert(norm(x(:, end) - xf) < 1.0, 'Mars landing final state error is too large.');

figure('Name', 'C-PDDP Mars Landing', 'Position', [140, 140, 1300, 760]);

subplot(2, 3, 1);
plot(horizontal_position, altitude, 'b-', 'LineWidth', 2); hold on;
plot(x0(1), x0(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Horizontal position (m)'); ylabel('Altitude (m)');
title('Descent Trajectory'); grid on; axis equal;
legend('Trajectory', 'Start', 'Landing site', 'Location', 'best');

subplot(2, 3, 2);
plot(t, altitude, 'b-', 'LineWidth', 2); hold on;
yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude'); grid on;

subplot(2, 3, 3);
plot(t, horizontal_velocity, 'LineWidth', 2); hold on;
plot(t, vertical_velocity, 'LineWidth', 2);
yline(0.0, 'k--');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity'); grid on;
legend('v_x', 'v_z', 'Location', 'best');

subplot(2, 3, 4);
stairs(t, u(1, :), 'LineWidth', 2); hold on;
stairs(t, u(2, :), 'LineWidth', 2);
yline(u_max(1), 'b--'); yline(-u_max(1), 'b--');
yline(u_max(2), 'r--'); yline(-u_max(2), 'r--');
xlabel('Time (s)'); ylabel('Acceleration command (m/s^2)');
title('Control Inputs'); grid on;
legend('a_x', 'a_z', 'Location', 'best');

subplot(2, 3, 5);
plot(t, sqrt(horizontal_velocity.^2 + vertical_velocity.^2), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed'); grid on;

subplot(2, 3, 6);
valid_cost = cost_history(cost_history ~= 0);
plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
xlabel('Iteration'); ylabel('Cost');
title('C-PDDP Cost History'); grid on;
set(gca, 'YScale', 'log');

sgtitle('Mars Powered Descent Optimal Control with C-PDDP');

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
