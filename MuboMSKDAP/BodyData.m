function [] = BodyData(i,q,qd)
%BodyData
%
%Summary: This function controls all actions regarding data for the rigid 
%         bodies, including its input, update and storage into local memory.
%
%Input:   i            - Body number
%         q            - Vector with the current system positions
%         qd           - Vector with current velocities
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%         NBody        - Number of Rigid Bodies in the system
%         Body         - All data for each Rigid Body in the system
%
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access memory
global H Body Flag Nline NCoordinates
%
%% ... Store initial positions for Rigid Bodies
if     Flag.ReadInput == 1
    Nline           = Nline + 1; 
    Body(i).r       = H(Nline,1:2)';
    Body(i).theta   = H(Nline,3);
    Body(i).rd      = H(Nline,4:5)';
    Body(i).thetad  = H(Nline,6);
    Body(i).mass    = H(Nline,7);
    Body(i).inertia = H(Nline,8);
%
%% ... Transfer coordinates from global to local storage
elseif Flag.Transfer == 1
    i1             = 3*i - 2;
    i2             = i1 + 1;
    i3             = i2 + 1;
%
%... Positions
    Body(i).r      = q(i1:i2,1);
    Body(i).theta  = q(i3:i3,1);
%
%... Velocities
    if (isempty(qd))
        Body(i).rd = zeros(size(Body(i).r));
        Body(i).rtheta = zeros(size(Body(i).theta));
    else
        Body(i).rd     = qd(i1:i2,1);
        Body(i).thetad = qd(i3:i3,1);
    end
%
%... Transformation and B matrices
    cost          = cos(Body(i).theta);
    sint          = sin(Body(i).theta);
    Body(i).A     = [ cost -sint;...
                      sint  cost];
    Body(i).B     = [-sint -cost;...
                      cost -sint];
end
%
%% ... Finish function BodyData
end
