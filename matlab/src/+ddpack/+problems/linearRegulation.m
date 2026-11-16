function problem = linearRegulation(varargin)
%LINEARREGULATION Create a deterministic linear regulation problem.
parser = inputParser;
addParameter(parser, 'InitialState', [1.2; 0], ...
    @(x) isnumeric(x) && iscolumn(x) && numel(x) == 2 && all(isfinite(x)));
parse(parser, varargin{:});

problem = struct();
problem.name = 'linear_regulation';
problem.x0 = parser.Results.InitialState;
problem.xf = [0; 0];
problem.tf = 15;
problem.dt = 0.3;
problem.dynamics = PointMassDynamics([0, 1; -1, -0.4], [0; 0.2], [0; 0]);
problem.cost = QuadraticCost(diag([200, 60]), diag([2, 0.5]), 0.5);
problem.u_max = 2;
problem.constraints = {};
problem.terminal_tolerance = 0.01;
problem.constraint_tolerance = 1e-8;
end
