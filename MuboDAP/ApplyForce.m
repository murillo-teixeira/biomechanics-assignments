function [g] = ApplyForce(f,spPi,Ai)
%ApplyForce
%
%Summary: This function adds an external force and its transport moment
%         to the body force vector.
%
%Input:   f        - External Force expressed in XY coordinates
%         spPi     - Point of application in body i (local coordinates)
%         Ai       - Transformation matrix
%
%Output:  g        - Body force vector
%
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Body force
g(1:2,1) = f;
fpi      = Ai'*f;
g(3  ,1) =-spPi(2,1)*fpi(1,1) + spPi(1,1)*fpi(2,1);
%
%% ... Finish function ApplyForce
end