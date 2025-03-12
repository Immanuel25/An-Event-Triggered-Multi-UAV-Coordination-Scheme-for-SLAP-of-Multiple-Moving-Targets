%% ======================================== Coordination pursuit controller ===================================================================
function [Tracker] = CoordinationCC(Tracker, kc,N)
    for i = 1:N
        % Find nearest neighbor in a cyclic manner (e.g., 1->2, 2->3, ..., N->1)
        neighborIdx = mod(i, N) + 1;

        % Apply consensus update
        Tracker(i).vc(end+1) = -kc * (Tracker(i).gamma(end) - Tracker(neighborIdx).gamma(end));
    end
    
    