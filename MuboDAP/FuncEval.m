function [yd] = FuncEval(t,y)
%
%Summary: Function that is invoked by the integrator. Performs
%         calculations and forms the auxiliary vector to be integrated.
%
%Input:   t     - Current analysis time
%         y     - Vector with state variables (positions and velocities)
%
%Output:  yd    - Vector with derivatives of state variables
% 
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access the global variables
%
global Flag NCoordinates NCoord1
global lambda acc
global tend w tstart count
%
%% ... Update the Waitbar
count = count + 1;
disp(count)
disp((t-tstart)/(tend-tstart))
w = waitbar((t-tstart)/(tend-tstart),w,['time: ',num2str(t,'%10.5f')]);
%
%% ... Form the Jacobian matrix, Acceleration rhs, Pos and Vel constraints
%
% ... Evaluate vectors and matrices
[ Phi,Jac,niu,gamma ] = KinemEval(t,y(1:NCoordinates,1),y(NCoord1:end,1));
%
%... Form the velocity constraint equations
Phid                  = Jac*y(NCoord1:end,1)-niu;
%
%% ... Create the force vectors of bodies and nodes
[g]   = Force(t);
%
%% ... Solve the system of the equations of motion
[acc,lambda] = SolveSystem(Phi,Jac,Phid,gamma,g);
%
%% ... Build the yd vector
yd    = [y(NCoord1:end,1); acc];
%
%% ... Finish function FuncEval
end