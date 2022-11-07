function [g] = ForceSprDamper(g)
%ForceSprDamper
%
%Summary: This function controls all actions regarding data for the  
%         Spring-Damper-Actuator forces, including its input, update and 
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
global Body
global H Frc Flag Nline
%
%% ... Store external force definition data
if     Flag.ReadInput == 1
    for k = 1:Frc.NSprDamper
        Nline                 = Nline + 1; 
        Frc.SprDamper(k).i    = H(Nline,1);
        Frc.SprDamper(k).j    = H(Nline,2);
        Frc.SprDamper(k).spPi = H(Nline,3:4)';
        Frc.SprDamper(k).spPj = H(Nline,5:6)';
        Frc.SprDamper(k).k    = H(Nline,7);
        Frc.SprDamper(k).l0   = H(Nline,8);
        Frc.SprDamper(k).c    = H(Nline,9);
        Frc.SprDamper(k).a    = H(Nline,10);
    end
    g = [];
%
%% ... Transfer velocities from global to local storage
elseif Flag.Acceleration == 1
    for k = 1:Frc.NSprDamper
        i  = Frc.SprDamper(k).i;
        j  = Frc.SprDamper(k).j;
%
% ... Evaluate the Spring force
        d  = Body(i).r + Body(i).A*Frc.SprDamper(k).spPi - ...
             Body(j).r - Body(j).A*Frc.SprDamper(k).spPj;
        l  = sqrt(d'*d);
        u  = d/l;
        fk = Frc.SprDamper(k).k*(l-Frc.SprDamper(k).l0);
%
% ... Evaluate the Damper force
        dd = Body(i).rd+Body(i).B*Frc.SprDamper(k).spPi*Body(i).thetad- ...
             Body(j).rd-Body(j).B*Frc.SprDamper(k).spPj*Body(j).thetad;
        ld = dd'*u;
        fd = Frc.SprDamper(k).c*ld;
%
% ... Evaluate the total force
        f  = (fk + fd + Frc.SprDamper(k).a)*u;
%
% ... Apply Spring-Damper-Actuator force to Body i
        i1 = 3*i-2;
        i3 = i1 + 2;
        g(i1:i3,1) = g(i1:i3,1) + ...
                     ApplyForce(-f,Frc.SprDamper(k).spPi,Body(i).A);
%
% ... Apply Spring-Damper-Actuator force to Body j
        i1 = 3*j-2;
        i3 = i1 + 2;
        g(i1:i3,1) = g(i1:i3,1) + ...
                     ApplyForce( f,Frc.SprDamper(k).spPj,Body(j).A);
    end
end
%
%% ... Finish function ForceSprDamper
end
