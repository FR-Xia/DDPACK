%% PDDP examples for non-UAV regulation problems
% Example 1: thermal error suppression in a cooling chamber.
% Example 2: vibration suppression of a lightly damped flexible mount.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== PDDP non-UAV regulation examples ===\n');

%% Example 1: thermal chamber cooling
rng(2);
dyn = PointMassDynamics(-0.2, 0.05, 0.0);
cost = QuadraticCost(200.0, 5.0, 1.0);
x0 = 2.0;
xf = 0.0;
solver_log = evalc('sol = pddp(x0, xf, 15.0, 0.3, dyn, cost, {}, 1.0, 80, 0.2);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 15.0);
final_error = abs(x(end) - xf);

fprintf('Thermal chamber final error: %.6f\n', final_error);
assert(sol.error == 0, 'PDDP thermal chamber example reported a numerical error.');
assert(final_error < 0.05, 'PDDP thermal chamber example did not converge enough.');

figure('Name', 'PDDP Thermal Chamber Regulation', 'Position', [80, 80, 1100, 720]);
subplot(2, 2, 1);
plot(t, x, 'LineWidth', 2); hold on; yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('Temperature error');
title('Thermal Error'); grid on;
subplot(2, 2, 2);
stairs(t, u, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Cooling command');
title('Control Input'); grid on;
subplot(2, 2, 3);
plot_valid_cost(J);
title('PDDP Cost History'); grid on;
subplot(2, 2, 4);
plot(t, abs(x), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('|error|');
title('Error Decay'); grid on;
sgtitle('PDDP Example 1: Cooling Chamber Regulation');

%% Example 2: flexible mount vibration suppression
rng(4);
A = [0.0, 1.0; -1.0, -0.4];
B = [0.0; 0.2];
dyn = PointMassDynamics(A, B, [0.0; 0.0]);
cost = QuadraticCost(diag([200.0, 60.0]), diag([2.0, 0.5]), 0.5);
x0 = [1.2; 0.0];
xf = [0.0; 0.0];
solver_log = evalc('sol = pddp(x0, xf, 15.0, 0.3, dyn, cost, {}, 2.0, 80, 0.2);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 15.0);
final_error = norm(x(:, end) - xf);

fprintf('Flexible mount final error: %.6f\n', final_error);
assert(sol.error == 0, 'PDDP vibration example reported a numerical error.');
assert(final_error < 0.01, 'PDDP vibration example did not converge enough.');

figure('Name', 'PDDP Flexible Mount Vibration Suppression', 'Position', [120, 120, 1200, 720]);
subplot(2, 2, 1);
plot(t, x(1, :), 'LineWidth', 2); hold on; yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('Displacement');
title('Displacement'); grid on;
subplot(2, 2, 2);
plot(t, x(2, :), 'LineWidth', 2); hold on; yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('Velocity');
title('Velocity'); grid on;
subplot(2, 2, 3);
stairs(t, u, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Actuator force');
title('Control Input'); grid on;
subplot(2, 2, 4);
plot_valid_cost(J);
title('PDDP Cost History'); grid on;
sgtitle('PDDP Example 2: Flexible Mount Vibration Suppression');

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

function plot_valid_cost(J)
    valid_cost = J(isfinite(J) & J ~= 0);
    plot(1:numel(valid_cost), valid_cost, 'LineWidth', 2);
    xlabel('Iteration'); ylabel('Cost');
    if numel(valid_cost) > 1 && all(valid_cost > 0)
        set(gca, 'YScale', 'log');
    end
end
