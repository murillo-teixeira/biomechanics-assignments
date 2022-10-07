function [ qdd ] = Acceleration_Analysis(time,q,qd,Jac)
%Acceleration_Analysis
%
%Summary: This function controls the kinematic acceleration analysis. The
%         method used is selected automatically by Matlab.
%
%Input:   time - Time instant in which the positions are evaluated
%         q    - System positions
%         qd   - System velocities
%         Jac  - Jacobian matrix
%
%Output:  qdd  - System accelerations
%
%%
%... Access the global memory
global Flag
global NBody Body
%
%%
%... Evaluate r.h.s of acceleration equations
    Flag.Acceleration = 1;
    [~,~,~,gamma] = KinemEval(time,q,qd);
%
%... Evaluate the accelerations
    qdd = Jac\gamma;
%
%... Transfer accelerations from global to local storage
    for i = 1:NBody
        i1 = 3*i - 2;
        i2 = i1 + 1;
        i3 = i2 + 1;
%
        Body(i).rdd     = qdd(i1:i2,1);
        Body(i).thetadd = qdd(i3:i3,1);
    end
%
%... Reset analysis flags
    Flag.Acceleration = 0;
%%
%... Finalize function Acceleration_Analysis
end