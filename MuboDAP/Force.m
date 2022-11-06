function [g]   = Force(t)
%
%Summary: Forms the force vector for the complete system
%
%Input:   t   - Current time at which the forces are ecvaluated
%
%Output:  g   - Force vector
% 
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Forces due to gravity
[g] = ForceGravity();
%
%% ... Forces due to gravity
[g] = ForceApplied(g);
%
%% ... Forces due to gravity
[g] = ForceSprDamper(g);
%
%% ... Forces that vary in time
[g] = VarForceApplied(g, t);
%
%% ... Finish function Force
end

