function raw = numericalFailureSolution(nominal,exception,diagnostics)
%NUMERICALFAILURESOLUTION Return a saved nominal without partial trial changes.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
if ~startsWith(exception.identifier,'ddpack:numerics:')
    rethrow(exception);
end
raw=nominal;raw.u{end}=raw.u{end-1};
raw.error=1;raw.termination_reason='numerical_failure';
raw.convergence_status='numerical_failure';
raw.numerical_diagnostics=diagnostics;
raw.numerical_diagnostics.failure_identifier=exception.identifier;
raw.numerical_diagnostics.failure_message=exception.message;
raw.J=raw.J(isfinite(raw.J));if isempty(raw.J),raw.J=0;end
raw.J_accepted=raw.J;
end
