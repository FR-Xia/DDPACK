%% FFT-CDDP examples for smooth non-UAV regulation problems
% Example 1: chemical reactor concentration regulation.
% Example 2: vibration isolation for a precision platform.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(repo_root, 'src')));

fprintf('=== FFT-CDDP smooth regulation examples ===\n');

%% Example 1: chemical reactor concentration regulation
dyn = PointMassDynamics(-0.30, 0.02, 0.0);
cost = QuadraticCost_fft(200.0, 2.0, 2.0);
x0 = 4.0;
xf = 0.0;
solver_log = evalc('sol = fft_cddp(x0, xf, 15.0, 50, dyn, cost, {}, 1.0, 80, 0.25);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 15.0);
final_error = abs(x(end) - xf);
tail_error = abs(x(max(1, end-10):end) - xf);

fprintf('Reactor concentration final error: %.6f\n', final_error);
assert(sol.error == 0, 'FFT-CDDP reactor example reported a numerical error.');
assert(final_error < 0.05, 'FFT-CDDP reactor example did not converge enough.');
assert(all(diff(tail_error) <= 1e-4), 'FFT-CDDP reactor terminal error increases near the end.');

figure('Name', 'FFT-CDDP Reactor Concentration Regulation', 'Position', [100, 100, 1100, 720]);
subplot(2, 2, 1);
plot(t, x, 'LineWidth', 2); hold on; yline(0.0, 'r--');
xlabel('Time (s)'); ylabel('Concentration error');
title('Concentration Error'); grid on;
subplot(2, 2, 2);
stairs(t, u, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Feed correction');
title('Control Input'); grid on;
subplot(2, 2, 3);
plot_valid_cost(J);
title('FFT-CDDP Cost History'); grid on;
subplot(2, 2, 4);
plot(t, abs(x), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('|error|');
title('Error Decay'); grid on;
sgtitle('FFT-CDDP Example 1: Reactor Concentration Regulation');

%% Example 2: precision platform vibration isolation
A = [0.0, 1.0; -1.0, -0.4];
B = [0.0; 0.2];
dyn = PointMassDynamics(A, B, [0.0; 0.0]);
cost = QuadraticCost_fft(diag([120.0, 40.0]), diag([1.0, 0.3]), 0.4);
x0 = [1.2; 0.0];
xf = [0.0; 0.0];
solver_log = evalc('sol = fft_cddp(x0, xf, 8.0, 40, dyn, cost, {}, 2.0, 80, 0.25);'); %#ok<NASGU>
[t, x, u, J] = collect_solution(sol, 8.0);
final_error = norm(x(:, end) - xf);

fprintf('Precision platform final error: %.6f\n', final_error);
assert(sol.error == 0, 'FFT-CDDP precision platform example reported a numerical error.');
assert(final_error < 0.25, 'FFT-CDDP precision platform example did not converge enough.');

figure('Name', 'FFT-CDDP Precision Platform Vibration Isolation', 'Position', [140, 140, 1200, 720]);
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
title('FFT-CDDP Cost History'); grid on;
sgtitle('FFT-CDDP Example 2: Precision Platform Vibration Isolation');

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
