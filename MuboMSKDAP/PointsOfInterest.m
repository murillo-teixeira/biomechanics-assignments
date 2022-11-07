function [] = PointsOfInterest(k)
%PointsOfInterest
%
%Summary: This function controls all actions regarding data the kinematics 
%         of the Points of Interest (PoI).
%
%Input:   k            - Time step number
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%         Pts.Int      - All data for each Point of Interest
%
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access memory
global H Body Flag Nline Ntime Pts
%
%% ... Store data for PoI
if     Flag.ReadInput == 1
    for n = 1:Pts.NPointsInt
        Nline           = Nline + 1; 
        Pts.Int(n).i    = H(Nline,1);
        Pts.Int(n).spPi = H(Nline,2:3)';
    end
%
%% ... Initialize memory for PoI
elseif Flag.InitData == 1
    for n = 1:Pts.NPointsInt
        Pts.Int(n).q(:,:)   = zeros(2,Ntime);
        Pts.Int(n).qd(:,:)  = zeros(2,Ntime);
        Pts.Int(n).qdd(:,:) = zeros(2,Ntime);
    end
%
%% ... Build the kinematics of the PoI
elseif Flag.Acceleration == 1
    for n = 1:Pts.NPointsInt
        i  = Pts.Int(n).i;
%
% ... Calculate and store the kinematics of the PoI
        Pts.Int(n).q(1:2,k)   = Body(i).r   + Body(i).A*Pts.Int(n).spPi;
        Pts.Int(n).qd(1:2,k)  = Body(i).rd  + Body(i).B*Pts.Int(n).spPi*...
                                              Body(i).thetad;
        Pts.Int(n).qdd(1:2,k) = Body(i).rdd + Body(i).B*Pts.Int(n).spPi*...
                                              Body(i).thetadd - ...
                                              Body(i).A*Pts.Int(n).spPi*...
                                              Body(i).thetad^2;
    end
end
%
%% ... Finish function PointsOfInterest
end
