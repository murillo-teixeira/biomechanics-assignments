function [ t,q,qd,qdd ] = Kinematic_Analysis( q0 )
%KinematicAnalysis 
%
%Summary: This function controls the kinematic analysis for a specified
%         duration and selected time-step.
%
%         Input: q0   - Estimate for the initial positions of all variables
%
%         Output: t   - Vector with the time of each time step analysis
%                 q   - Position history
%                 qd  - Position history
%                 qdd - Position history
%
%%
%... Acess the global memory
global tstart tstep tend NCoordinates Ntime Pts Body
%%
%... Create the workspaces
q   = zeros(NCoordinates,Ntime);
qd  = zeros(NCoordinates,Ntime);
qdd = zeros(NCoordinates,Ntime);
%
for k = 1:Pts.NPointsInt
    Pts.Int(k).q(:,:)   = zeros(2,Ntime);
    Pts.Int(k).qd(:,:)  = zeros(2,Ntime);
    Pts.Int(k).qdd(:,:) = zeros(2,Ntime);
end
%
k   = 0;
%%
%... For each time step perform the complete kinematic analysis
for time = tstart:tstep:tend
    k = k+1;
    q(:,k) = q0;
    %
    %... Position analysis
    [q(:,k),Jac] = Position_Analysis(time,q(:,k));
    %
    %... Velocity analysis
    [qd(:,k)]    = Velocity_Analysis(time,q(:,k),Jac);
    %
    %... Acceleration analysis
    [qdd(:,k)]   = Acceleration_Analysis(time,q(:,k),qd(:,k),Jac);
    %
    %... Calculate the kinematics of the points of interest
    for n = 1:Pts.NPointsInt
        i  = Pts.Int(n).i;
        Pts.Int(n).q(1:2,k)   = Body(i).r   + Body(i).A*Pts.Int(n).spPi;
        Pts.Int(n).qd(1:2,k)  = Body(i).rd  + Body(i).B*Pts.Int(n).spPi*...
                                              Body(i).thetad;
        Pts.Int(n).qdd(1:2,k) = Body(i).rdd + Body(i).B*Pts.Int(n).spPi*...
                                              Body(i).thetadd - ...
                                              Body(i).A*Pts.Int(n).spPi*...
                                              Body(i).thetad^2;
    end
    %
    %... Use previous time step positions as estimates for next time step
    q0 = q(:,k);
end
t   = tstart:tstep:tend;
%%
%... Finalize function Kinematic_Analysis
end

