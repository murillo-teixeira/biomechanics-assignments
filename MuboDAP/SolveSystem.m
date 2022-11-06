function [qdd,lambda] = SolveSystem(Phi,Jac,Phid,gamma,g)
%
%Summary: Solves the system of equations to obtain the system acceleration
%         and the Lagrange multipliers
%
%Input:   Phi   - Position constraint equations
%         Jac   - Jacobian matrix
%         Phid  - Velocity constraint equations
%         gamma - RHS of the acceleration constraint equations
%         g     - Force vector
%
%Output:  yd    - Vector with derivatives of state variables
% 
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access the global variables
%
global M Minv NCoordinates NCoord1 NConstraints
global parameters
%
%% ... Solution using the matlab equation solvers directly
if (parameters.MotionEqSolver == 1)
%    
% ... Form the leading matrix
    A   = [M   Jac'; ...
           Jac zeros(NConstraints,NConstraints)];
%
% ... Form the RHS vector
    b   = [g; ...
           gamma-2*parameters.alpha*Phid-parameters.beta*Phi];
%
% ... Solve system of equations
    x   = A\b;
%
% ... Return the vectors with accelerations and Lagrange multipliers
    qdd    = x(1:NCoordinates,1);
    lambda = x(NCoord1:end,1);
%
%% ... Solution using the structure of the equations
elseif (parameters.MotionEqSolver == 2)
%    
% ... Form the leading matrix
    A      = Jac*Minv*Jac';
%
% ... Form the RHS vector
    b      = Jac*Minv*g-gamma+2*parameters.alpha*Phid+parameters.beta*Phi;
%
% ... Evaluate the vectors with accelerations and Lagrange multipliers
    lambda = A\b;
    qdd    = Minv*(g-Jac'*lambda);
end
%
%% ... Finish function SolveSystem
end

