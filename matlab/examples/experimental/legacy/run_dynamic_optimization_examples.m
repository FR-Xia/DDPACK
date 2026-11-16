%% Run all non-UAV dynamic optimization examples.

clear; clc; close all;

example_dir = fileparts(mfilename('fullpath'));

fprintf('Running dynamic optimal-control examples...\n\n');

run(fullfile(example_dir, 'example_pddp_regulation_problems.m'));
fprintf('\n');

run(fullfile(fileparts(mfilename('fullpath')), 'example_mspddp_ground_robot_tasks.m'));
fprintf('\n');

run(fullfile(fileparts(mfilename('fullpath')), 'example_fft_cddp_smooth_regulation.m'));
fprintf('\n');

run(fullfile(fileparts(mfilename('fullpath')), 'example_cbf_mspddp_safety_tasks.m'));
fprintf('\n');

run(fullfile(example_dir, 'example_inverted_pendulum.m'));
fprintf('\n');

run(fullfile(fileparts(mfilename('fullpath')), 'example_c_pddp_rocket_ascent.m'));
fprintf('\n');

run(fullfile(fileparts(mfilename('fullpath')), 'example_c_pddp_mars_landing.m'));
fprintf('\nAll dynamic optimal-control examples completed successfully.\n');
