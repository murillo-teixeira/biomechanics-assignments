function [g] = ForceApplied(g)
%ForceApplied
%
%Summary: This function controls all actions regarding data for the  
%         external applied forces, including its input, update and 
%         storage into local memory.
%
%Input:   g            - Vector with system forces
%
%Output:  g            - Vector with system forces
%
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access memory
global H Frc Flag Nline
global Body
%
%% ... Store external force definition data
if     Flag.ReadInput == 1
    for k = 1:Frc.NForceAppl
        Nline                 = Nline + 1; 
        Frc.ForceAppl(k).i    = H(Nline,1);
        Frc.ForceAppl(k).f    = H(Nline,2:3)';
        Frc.ForceAppl(k).n    = H(Nline,4);
        Frc.ForceAppl(k).spPi = H(Nline,5:6)';
    end
    g = [];
%
%% ... Transfer velocities from global to local storage
elseif Flag.Acceleration == 1
    for k = 1:Frc.NForceAppl
        i  = Frc.ForceAppl(k).i;
        i1 = 3*i - 2;
        i3 = i1 + 2;
%
        g(i1:i3,1) = g(i1:i3,1) + ...
                     ApplyForce(Frc.ForceAppl(k).f,...
                                Frc.ForceAppl(k).spPi,Body(i).A);
        g(i3:i3,1) = g(i3:i3,1) + Frc.ForceAppl(k).n;
    end
end
%
%% ... Finish function ForceApplied
end
