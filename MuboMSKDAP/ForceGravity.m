function [g] = ForceGravity()
%
%Summary: Forms the gravity force vector for the complete system
%
%Input:   None
%
%Output:  g   - Force vector
% 
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access the global variables
global NBody Body gravity NCoordinates
%
%% ... Initialize the force vector
g = zeros(NCoordinates,1);
%
% ... Add the gravity force for each rigid body
for i=1:NBody
    i1 = 3*i - 2;
    i2 = i1 + 1;
    g(i1:i2,1) = Body(i).mass*gravity;
end
%
%% ... Finish function ForceGravity
end