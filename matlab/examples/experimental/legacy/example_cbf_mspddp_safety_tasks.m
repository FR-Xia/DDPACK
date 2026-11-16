%% CBF-MSPDDP examples for safety-constrained non-UAV tasks
% Example 1: hospital delivery robot avoiding a sterile zone.
% Example 2: cleanroom inspection robot avoiding a restricted machine area.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== CBF-MSPDDP safety-constrained examples ===\n');

%% Example 1: hospital delivery robot
dyn = Dynamics2D_cbf_mspddp(1.0);
cost = QuadraticCost(diag([200.0, 200.0, 40.0]), diag([1.0, 1.0, 0.2]), 0.2);
x0 = [0.0; 0.0; 0.0];
xf = [5.0; 0.0; 0.0];
obs_position = [2.5; 0.10];
obs_radius = 0.12;
solver_log = evalc('sol = cbf_mspddp_2d(x0, xf, 10.0, 0.2, dyn, cost, 1.5, 80, 0.2, 0.05, 0.95, obs_position, obs_radius, 0.5, 1, 1);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 10.0);
final_error = norm(x(:, end) - xf);
min_distance = min(hypot(x(1, :) - obs_position(1), x(2, :) - obs_position(2)));
straight_clearance = point_to_segment_distance(obs_position, x0(1:2), xf(1:2));
straight_deviation = max(abs(x(2, :) - linspace(x0(2), xf(2), numel(sol.x))));

fprintf('Hospital delivery final error: %.6f, min distance: %.6f, path deviation: %.6f\n', ...
    final_error, min_distance, straight_deviation);
assert(sol.error == 0, 'CBF-MSPDDP hospital example reported a numerical error.');
assert(final_error < 0.35, 'CBF-MSPDDP hospital example did not converge enough.');
assert(min_distance > obs_radius, 'CBF-MSPDDP hospital example violates obstacle clearance.');
assert(straight_clearance < obs_radius, 'Hospital straight-line path does not intersect the safety zone.');
assert(straight_deviation > 0.15, 'CBF-MSPDDP hospital path does not show visible avoidance.');

figure('Name', 'CBF-MSPDDP Hospital Delivery Robot', 'Position', [100, 100, 1200, 720]);
plot_cbf_case(t, x, u, J, x0, xf, obs_position, obs_radius, 'CBF-MSPDDP Example 1: Hospital Delivery Robot');

%% Example 2: cleanroom inspection robot
dyn = Dynamics2D_cbf_mspddp(1.0);
cost = QuadraticCost(diag([200.0, 200.0, 40.0]), diag([1.0, 1.0, 0.2]), 0.2);
x0 = [0.0; 1.0; 0.0];
xf = [5.0; 1.0; 0.0];
obs_position = [2.5; 1.10];
obs_radius = 0.12;
solver_log = evalc('sol = cbf_mspddp_2d(x0, xf, 10.0, 0.2, dyn, cost, 1.5, 80, 0.2, 0.05, 0.95, obs_position, obs_radius, 0.5, 1, 1);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 10.0);
final_error = norm(x(:, end) - xf);
min_distance = min(hypot(x(1, :) - obs_position(1), x(2, :) - obs_position(2)));
straight_clearance = point_to_segment_distance(obs_position, x0(1:2), xf(1:2));
straight_deviation = max(abs(x(2, :) - linspace(x0(2), xf(2), numel(sol.x))));

fprintf('Cleanroom inspection final error: %.6f, min distance: %.6f, path deviation: %.6f\n', ...
    final_error, min_distance, straight_deviation);
assert(sol.error == 0, 'CBF-MSPDDP cleanroom example reported a numerical error.');
assert(final_error < 0.35, 'CBF-MSPDDP cleanroom example did not converge enough.');
assert(min_distance > obs_radius, 'CBF-MSPDDP cleanroom example violates obstacle clearance.');
assert(straight_clearance < obs_radius, 'Cleanroom straight-line path does not intersect the safety zone.');
assert(straight_deviation > 0.15, 'CBF-MSPDDP cleanroom path does not show visible avoidance.');

figure('Name', 'CBF-MSPDDP Cleanroom Inspection Robot', 'Position', [140, 140, 1200, 720]);
plot_cbf_case(t, x, u, J, x0, xf, obs_position, obs_radius, 'CBF-MSPDDP Example 2: Cleanroom Inspection Robot');

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

function plot_cbf_case(t, x, u, J, x0, xf, obs_position, obs_radius, title_text)
    subplot(2, 2, 1);
    plot(x(1, :), x(2, :), 'b-', 'LineWidth', 2); hold on;
    plot([x0(1), xf(1)], [x0(2), xf(2)], 'k--', 'LineWidth', 1.2);
    plot(x(1, 1), x(2, 1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(xf(1), xf(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    th = linspace(0, 2*pi, 80);
    fill(obs_position(1) + obs_radius*cos(th), obs_position(2) + obs_radius*sin(th), ...
        [1.0, 0.8, 0.8], 'EdgeColor', 'r', 'FaceAlpha', 0.55);
    xlabel('x (m)'); ylabel('y (m)');
    title('Workspace Path and Safety Zone'); grid on; axis equal;
    legend('Optimized', 'Blocked straight path', 'Start', 'Goal', 'Restricted zone', 'Location', 'best');

    subplot(2, 2, 2);
    plot(t, x(3, :), 'LineWidth', 2); hold on; yline(xf(3), 'r--');
    xlabel('Time (s)'); ylabel('Heading (rad)');
    title('Heading'); grid on;

    subplot(2, 2, 3);
    stairs(t, u, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Angular rate');
    title('Control Input'); grid on;

    subplot(2, 2, 4);
    plot_valid_cost(J);
    title('CBF-MSPDDP Cost History'); grid on;
    sgtitle(title_text);
end

function d = point_to_segment_distance(p, a, b)
    ab = b - a;
    tau = max(0.0, min(1.0, dot(p - a, ab) / dot(ab, ab)));
    projection = a + tau * ab;
    d = norm(p - projection);
end

function plot_valid_cost(J)
    valid_cost = J(isfinite(J) & J ~= 0);
    plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
    xlabel('Iteration'); ylabel('Cost');
    if numel(valid_cost) > 1 && all(valid_cost > 0)
        set(gca, 'YScale', 'log');
    end
end
