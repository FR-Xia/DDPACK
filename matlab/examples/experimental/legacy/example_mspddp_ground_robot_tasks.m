%% MSPDDP examples for non-UAV ground robot tasks
% Example 1: automated guided vehicle lane-change transfer.
% Example 2: pipe inspection robot sidewall approach.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== MSPDDP ground robot examples ===\n');

%% Example 1: automated guided vehicle lane-change transfer
% The platform advances down the aisle at nominal speed and optimizes one
% lateral actuator to settle into the target lane.
dyn = PointMassDynamics(zeros(3), [0.0; 1.0; 0.0], [1.0; 0.0; 0.0]);
cost = QuadraticCost(diag([300.0, 300.0, 20.0]), diag([1.0, 2.0, 0.1]), 0.2);
x0 = [0.0; 0.0; 0.0];
xf = [5.0; 1.2; 0.0];
solver_log = evalc('sol = mspddp(x0, xf, 10.0, 0.2, dyn, cost, 2.0, 80, 0.2, 0.05, 0.95);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 10.0);
final_error = norm(x(:, end) - xf);
straight_deviation = max(abs(x(2, :) - linspace(x0(2), xf(2), numel(sol.x))));

fprintf('AGV transfer final error: %.6f, path deviation: %.6f\n', final_error, straight_deviation);
assert(sol.error == 0, 'MSPDDP AGV example reported a numerical error.');
assert(final_error < 1e-3, 'MSPDDP AGV example did not converge enough.');
assert(straight_deviation > 0.25, 'MSPDDP AGV example is too close to a straight line.');

figure('Name', 'MSPDDP AGV Lane-Change Transfer', 'Position', [100, 100, 1200, 720]);
plot_ground_robot_case(t, x, u, J, x0, xf, 'MSPDDP Example 1: AGV Lane-Change Transfer');

%% Example 2: pipe inspection robot sidewall approach
dyn = PointMassDynamics(zeros(3), [0.0; 1.0; 0.0], [1.0; 0.0; 0.0]);
cost = QuadraticCost(diag([300.0, 300.0, 20.0]), diag([1.0, 2.0, 0.1]), 0.2);
x0 = [0.0; 1.0; 0.0];
xf = [5.0; -1.0; 0.0];
solver_log = evalc('sol = mspddp(x0, xf, 10.0, 0.2, dyn, cost, 2.0, 80, 0.2, 0.05, 0.95);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 10.0);
final_error = norm(x(:, end) - xf);
straight_deviation = max(abs(x(2, :) - linspace(x0(2), xf(2), numel(sol.x))));

fprintf('Pipe inspection final error: %.6f, path deviation: %.6f\n', final_error, straight_deviation);
assert(sol.error == 0, 'MSPDDP pipe inspection example reported a numerical error.');
assert(final_error < 1e-3, 'MSPDDP pipe inspection example did not converge enough.');
assert(straight_deviation > 0.25, 'MSPDDP pipe inspection example is too close to a straight line.');

figure('Name', 'MSPDDP Pipe Inspection Sidewall Approach', 'Position', [140, 140, 1200, 720]);
plot_ground_robot_case(t, x, u, J, x0, xf, 'MSPDDP Example 2: Pipe Inspection Sidewall Approach');

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

function plot_ground_robot_case(t, x, u, J, x0, xf, title_text)
    subplot(2, 2, 1);
    plot(x(1, :), x(2, :), 'b-', 'LineWidth', 2); hold on;
    plot([x0(1), xf(1)], [x0(2), xf(2)], 'k--', 'LineWidth', 1.2);
    plot(x(1, 1), x(2, 1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(xf(1), xf(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    xlabel('x (m)'); ylabel('y (m)');
    title('Workspace Path'); grid on; axis equal;
    legend('Optimized', 'Straight reference', 'Start', 'Goal', 'Location', 'best');

    subplot(2, 2, 2);
    plot(t, x(2, :), 'LineWidth', 2); hold on; yline(xf(2), 'r--');
    xlabel('Time (s)'); ylabel('Lateral offset (m)');
    title('Lateral Offset'); grid on;

    subplot(2, 2, 3);
    stairs(t, u, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Lateral rate command');
    title('Control Input'); grid on;

    subplot(2, 2, 4);
    plot_valid_cost(J);
    title('MSPDDP Cost History'); grid on;
    sgtitle(title_text);
end

function plot_valid_cost(J)
    valid_cost = J(isfinite(J) & J ~= 0);
    plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
    xlabel('Iteration'); ylabel('Cost');
    if numel(valid_cost) > 1 && all(valid_cost > 0)
        set(gca, 'YScale', 'log');
    end
end
