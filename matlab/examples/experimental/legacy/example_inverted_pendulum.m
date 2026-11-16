%% Inverted pendulum optimal control using existing C-PDDP
% State:   x = [theta; theta_dot]
% Control: u = motor torque
% Goal: swing from downward equilibrium to upright balance.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== Inverted pendulum optimal-control example (C-PDDP) ===\n');

m = 1.0;
l = 1.0;
g = 9.81;
b = 0.1;

dyn = InvertedPendulumDynamics(m, l, g, b);

x0 = [pi; 0.0];
xf = [0.0; 0.0];
tf = 5.0;
dt = 0.05;
u_max = 10.0;
num_iter = 120;
alpha = 0.4;

Q_f = diag([100.0, 10.0]);
Q_s = diag([1.0, 0.1]);
R = 0.1;
cost = InvertedPendulumCost(Q_f, Q_s, R, xf);

solver_log = evalc('[sol, sol_init] = c_pddp(0, 0, x0, [], xf, tf, dt, dyn, cost, u_max, num_iter, alpha);'); %#ok<NASGU,ASGLU>

[t, x, u, cost_history] = collect_solution(sol, tf);
theta = wrapToPiLocal(x(1, :));
theta_dot = x(2, :);
torque = u(1, :);
final_error = norm([theta(end); theta_dot(end)] - xf);

fprintf('Algorithm: C-PDDP\n');
fprintf('Final angle: %.6f rad\n', theta(end));
fprintf('Final angular velocity: %.6f rad/s\n', theta_dot(end));
fprintf('Final state error: %.6f\n', final_error);
fprintf('Peak torque: %.6f N*m\n', max(abs(torque)));

assert(sol.error == 0, 'C-PDDP reported a numerical error.');
assert(final_error < 1e-2, 'Inverted pendulum final state error is too large.');

figure('Name', 'C-PDDP Inverted Pendulum', 'Position', [100, 100, 1300, 760]);

subplot(2, 3, 1);
plot(t, theta, 'b-', 'LineWidth', 2); hold on;
yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Pendulum Angle'); grid on;

subplot(2, 3, 2);
plot(t, theta_dot, 'm-', 'LineWidth', 2); hold on;
yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('\theta dot (rad/s)');
title('Angular Velocity'); grid on;

subplot(2, 3, 3);
stairs(t, torque, 'k-', 'LineWidth', 2); hold on;
yline(u_max, 'r--'); yline(-u_max, 'r--');
xlabel('Time (s)'); ylabel('Torque (N*m)');
title('Control Input'); grid on;

subplot(2, 3, 4);
plot(theta, theta_dot, 'b-', 'LineWidth', 2); hold on;
plot(theta(1), theta_dot(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(theta(end), theta_dot(end), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('\theta (rad)'); ylabel('\theta dot (rad/s)');
title('Phase Portrait'); grid on;
legend('Trajectory', 'Start', 'Target', 'Location', 'best');

subplot(2, 3, 5);
valid_cost = cost_history(cost_history ~= 0);
plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
xlabel('Iteration'); ylabel('Cost');
title('C-PDDP Cost History'); grid on;
set(gca, 'YScale', 'log');

subplot(2, 3, 6);
plot_pendulum_snapshots(theta, l);
title('Pendulum Motion Snapshots');

sgtitle('Inverted Pendulum Swing-up with C-PDDP');

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

function y = wrapToPiLocal(x)
    y = mod(x + pi, 2*pi) - pi;
end

function plot_pendulum_snapshots(theta, l)
    hold on; axis equal; grid on;
    idx = round(linspace(1, numel(theta), 8));
    cmap = parula(numel(idx));
    for i = 1:numel(idx)
        th = theta(idx(i));
        px = l * sin(th);
        py = l * cos(th);
        plot([0, px], [0, py], '-', 'Color', cmap(i, :), 'LineWidth', 2);
        plot(px, py, 'o', 'Color', cmap(i, :), 'MarkerFaceColor', cmap(i, :), 'MarkerSize', 5);
    end
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    xlim([-1.2*l, 1.2*l]);
    ylim([-1.2*l, 1.2*l]);
    xlabel('x (m)'); ylabel('y (m)');
end
