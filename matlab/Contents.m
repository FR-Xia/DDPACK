% DDPACK 1.0.0 - Differential-dynamic-programming optimization package.
%
% Setup
%   setup_ddpack            - Add formal source folders to the MATLAB path.
%
% Public API
%   ddpack.solve            - Solve and normalize an optimal-control problem.
%   ddpack.Options          - Validated algorithm options.
%   ddpack.algorithmRegistry - Return authoritative algorithm identities.
%   ddpack.feasibilityFromLessEqual - Convert c <= 0 to G >= 0.
%   ddpack.plot             - Plot a normalized solution.
%   ddpack.save             - Save a versioned solution.
%   ddpack.load             - Load and validate a solution.
%   ddpack.version          - Return package version.
%   DDPACK                  - Backward-compatible object interface.
%
% Verification
%   run_publication_examples - Run stable examples and create artifacts.
%   run_all_tests            - Run unit, integration, and regression tests.
%   run_benchmarks           - Run warmed, repeated stable-scope benchmarks.
%   check_manuscript         - Compile and validate the SoftwareX manuscript.
%   create_release           - Build a versioned source archive and manifest.
%   run_final_softwarex_acceptance - Execute the single formal acceptance flow.
