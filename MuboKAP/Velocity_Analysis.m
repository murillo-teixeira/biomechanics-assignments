function [ qd ] = Velocity_Analysis(time,q,Jac)
%Velocity_Analysis
%
%Summary: This function controls the kinematic velocity analysis. The
%         method used is selected automatically by Matlab.
%
%Input:   time - Time instant in which the positions are evaluated
%         q    - System positions
%         Jac  - Jacobian matrix
%
%Output:  qd   - System velocities
%
%%
%... Access the global memory
global Flag
%
%%
%... Evaluate r.h.s of velocity equations
    Flag.Transfer = 1;
    Flag.Velocity = 1;
    [~,~,niu,~] = KinemEval(time,q,[]);
%
%... Evaluate the velocities
    qd = Jac\niu;
%
%... Reset analysis flags
    Flag.Transfer = 0;
    Flag.Velocity = 0;
%%
%... Finalize function Velocity_Analysis
end