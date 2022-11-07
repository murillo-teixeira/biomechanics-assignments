function [Phi,Jac,niu,gamma] = Joint_Driver (Phi,Jac,niu,gamma,k,time)
%Joint_Revolute
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a Driver.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the Driver
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Driver - Driver information
%
%
%% ... Access global memory
global H NConstraints lambda
global Flag Nline Body Jnt NJoint
global spline_type
%
%% ... Read input for this joint
if Flag.ReadInput == 1
    Nline                  = Nline + 1;
    Jnt.Driver(k).type     = H(Nline,1);
    Jnt.Driver(k).i        = H(Nline,2);
    Jnt.Driver(k).coortype = H(Nline,3);
    Jnt.Driver(k).j        = H(Nline,4);
%
    switch Jnt.Driver(k).type
        case 1
            Jnt.Driver(k).z0       = H(Nline,5); % Initial position
            Jnt.Driver(k).vel      = H(Nline,6); % Initial velocity
            Jnt.Driver(k).acc      = H(Nline,7); % Acceleration
        case 2
            Jnt.Driver(k).z0       = H(Nline,5); % Initial position
            Jnt.Driver(k).zmin     = H(Nline,6); % Minimum of the range
            Jnt.Driver(k).zmax     = H(Nline,7); % Maximum of the range
            Jnt.Driver(k).freq     = H(Nline,8); % Frequency
        case {3,4,5}
            Jnt.Driver(k).spPi     = H(Nline,5:6)';
            Jnt.Driver(k).spPj     = H(Nline,7:8)';
            Jnt.Driver(k).order    = H(Nline,9);  % order of spline
            x                      = H(Nline,10); % File for actuator input
            Jnt.Driver(k).Filename = sprintf('Driver_%03d.txt',x);
    end
    return
end
%
%% ... Initialize data for the driver
if Flag.InitData == 1
    NConstraints      = NConstraints + 1;
%
    switch Jnt.Driver(k).type
        case 2
            Jnt.Driver(k).zavg = (Jnt.Driver(k).zmax+Jnt.Driver(k).zmin)/2;
            Jnt.Driver(k).zamp = (Jnt.Driver(k).zmax-Jnt.Driver(k).zmin)/2;
            Jnt.Driver(k).beta = asin((Jnt.Driver(k).z0 - ...
                                       Jnt.Driver(k).zavg) / ...
                                       Jnt.Driver(k).zamp);
        case {3,4,5}
%
%... Read data from input file
            H   = dlmread([cd, '\', Jnt.Driver(k).Filename]);
%
%... Evaluate the spline interpolation and its derivatives
            spline_type             = 1;
            order                   = Jnt.Driver(k).order;
            Jnt.Driver(k).Spline    = DSM_spline(H(:,1)',H(:,2)',order);
            Jnt.Driver(k).Splined   = ppdiff(Jnt.Driver(k).Spline, 1);
            Jnt.Driver(k).Splinedd  = ppdiff(Jnt.Driver(k).Splined, 1);
%             Jnt.Driver(k).Splined   = fnder(Jnt.Driver(k).Spline);
%             Jnt.Driver(k).Splinedd  = fnder(Jnt.Driver(k).Splined);
%
%... For the angle driver find the initial angle between the two vectors
            if (Jnt.Driver(k).type == 4)
                if (norm(Jnt.Driver(k).spPi) == 0)
                    Jnt.Driver(k).angleij = 0;
                else
                    spPi                  = Jnt.Driver(k).spPi;
                    spPj                  = Jnt.Driver(k).spPj;
                    uspPi                 = spPi/sqrt(spPi'*spPi);
                    uspPj                 = spPj/sqrt(spPj'*spPj);
                    Jnt.Driver(k).angleij = atan2(uspPi(1,1),uspPi(2,1)) - ...
                        atan2(uspPj(1,1),uspPj(2,1));
                end
            end
    end
%
%... Prepare Joint Reactions Report
%
    NJoint                      = Jnt.NReaction + 1;
    Jnt.NReaction               = NJoint;
    Jnt.Reaction(NJoint).Number = k;
    Jnt.Reaction(NJoint).i      = Jnt.Driver(k).i;
    Jnt.Reaction(NJoint).Type   = '.........Driver';
    switch Jnt.Driver(k).type
        case {1,2}
            Jnt.Reaction(NJoint).j  = 0;
        case {3,4,5}
            Jnt.Reaction(NJoint).j  = Jnt.Driver(k).j;
    end
    return
end
%
%% ... Line numbers of constraint equations & Pointer of next constraints
if Flag.General == 1
    i1      = Nline;
    Nline   = Nline + 1;
%
%... Initialize variables
    i     = Jnt.Driver(k).i;
    type  = Jnt.Driver(k).type;
    coord = Jnt.Driver(k).coortype;
    switch type
        case 1
            z0    = Jnt.Driver(k).z0;
            v     = Jnt.Driver(k).vel;
            a     = Jnt.Driver(k).acc;
        case 2
            zavg  = Jnt.Driver(k).zavg;
            zamp  = Jnt.Driver(k).zamp;
            freq  = Jnt.Driver(k).freq;
            beta  = Jnt.Driver(k).beta;
        case {3,4,5}
            j     = Jnt.Driver(k).j;
            z     = ppval(Jnt.Driver(k).Spline,time);
            if (type == 5)
                d     = Body(i).r + Body(i).A*Jnt.Driver(k).spPi - ...
                        Body(j).r - Body(j).A*Jnt.Driver(k).spPj;
            end
    end
end
%
%% ... Assemble position constraint equations
if Flag.Position == 1 
    if type < 4
        switch coord
            case 1
                aux = Body(i).r(1,1);
            case 2
                aux = Body(i).r(2,1);
            case 3
                aux = Body(i).theta;
        end
    end
%
    switch type
        case 1
            Phi(i1:i1,1) = aux - (z0 + v*time + 0.5*a*time^2);
        case 2
            Phi(i1:i1,1) = aux - (zavg + zamp*sin(freq*time+beta));
        case 3
            Phi(i1:i1,1) = aux - z;
        case 4
            Phi(i1:i1,1) = Body(j).theta - Body(i).theta - z - ...
                           Jnt.Driver(k).angleij;
        case 5
            Phi(i1:i1,1) = d'*d - z^2;
    end
end
%
%% ... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    switch type
        case {1, 2, 3}
            j1 = 3*(i-1) + coord;
            Jac(i1:i1,j1:j1) = 1.0;
        case 4
            j1 = 3*i;
            Jac(i1:i1,j1:j1) = -1.0;
            j1 = 3*j;
            Jac(i1:i1,j1:j1) =  1.0;
        case 5
            j1 = 3*i - 2;
            j2 = j1 + 2;
            j3 = 3*j - 2; 
            j4 = j3 + 2;
            Jac(i1:i1,j1:j2) =2*[ d', d'*Body(i).B*Jnt.Driver(k).spPi];
            Jac(i1:i1,j3:j4) =2*[-d',-d'*Body(j).B*Jnt.Driver(k).spPj];
    end
end
%
%% ... Contribution to the r.h.s. of the velocity equations is null
if Flag.Velocity == 1
    switch type
        case 1
            niu(i1:i1,1) = v + a*time;
        case 2
            niu(i1:i1,1) = zamp*freq*cos(freq*time+beta);
        case {3, 4}
            niu(i1:i1,1) = ppval(Jnt.Driver(k).Splined,time);
        case 5
            niu(i1:i1,1) = 2.0*z*ppval(Jnt.Driver(k).Splined,time);
    end
end
%
%% ... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    switch type
        case 1
            gamma(i1:i1,1) = a;
        case 2
            gamma(i1:i1,1) =-zamp*freq^2*sin(freq*time+beta);
        case {3,4}
            gamma(i1:i1,1) = ppval(Jnt.Driver(k).Splinedd,time);
        case 5
            tid = Body(i).thetad;
            tjd = Body(j).thetad;
            dd  = Body(i).rd + Body(i).B*Jnt.Driver(k).spPi*tid - ...
                  Body(j).rd - Body(j).B*Jnt.Driver(k).spPj*tjd;
            gamma(i1:i1,1) =-dd'*dd + ...
                             d'*(Body(i).A*Jnt.Driver(k).spPi*tid^2 - ...
                                 Body(j).A*Jnt.Driver(k).spPj*tjd^2) + ...
                             2.0*(ppval(Jnt.Driver(k).Splined ,time)^2+ ...
                               z* ppval(Jnt.Driver(k).Splinedd,time));
    end
end
%
%% ... Evaluate the Joint Reaction Forces for Report
if Flag.Reaction == 1
    NJoint                  = NJoint +1;
    j1                      = 3*i - 2;
    j2                      = j1 + 2;
    j3                      = 3*j - 2;
    j4                      = j3 + 2;
    Jnt.Reaction(NJoint).gi = -Jac(i1:i1,j1:j2)'*lambda(i1:i1,1);
    switch type
        case {4,5}
            Jnt.Reaction(NJoint).gj = -Jac(i1:i1,j3:j4)'*lambda(i1:i1,1);
    end
end
%
%% ... Finish function Joint_Revolute
end
  