function [acceptedCosts, acceptedIndices] = strictAcceptedCosts(trialCosts)
%STRICTACCEPTEDCOSTS Return strict best-so-far finite trial costs.
validateattributes(trialCosts, {'numeric'}, {'vector', 'real'});
trialCosts = trialCosts(:);
acceptedCosts = zeros(0, 1);
acceptedIndices = zeros(0, 1);
best = Inf;
for index = 1:numel(trialCosts)
    candidate = trialCosts(index);
    if isfinite(candidate) && candidate < best
        best = candidate;
        acceptedCosts(end + 1, 1) = candidate; %#ok<AGROW>
        acceptedIndices(end + 1, 1) = index; %#ok<AGROW>
    end
end
end
