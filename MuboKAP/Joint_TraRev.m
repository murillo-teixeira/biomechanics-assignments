function [Phi,Jac,niu,gamma] = Joint_TraRev (Phi,Jac,niu,gamma,k,time)
%Joint_Revolute
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a 
%         composite Translation-Revolute joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the Translation-Revolute joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.TraRev - Translation-Revolute joint information
%
%
%%
%... Access global memory
global H NConstraints A90
global Flag Nline Body Jnt
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.TraRev(k).i    = H(Nline,1);
    Jnt.TraRev(k).j    = H(Nline,2);
    Jnt.TraRev(k).l0   = H(Nline,3);
    Jnt.TraRev(k).spPi = H(Nline,4:5)';
    Jnt.TraRev(k).spPj = H(Nline,6:7)';
    Jnt.TraRev(k).spQj = H(Nline,8:9)';
    return
end
%%
%... Initialize data for this joint
if Flag.InitData == 1
    NConstraints      = NConstraints + 1;
    Jnt.TraRev(k).hpj = A90*(Jnt.TraRev(k).spPj - Jnt.TraRev(k).spQj);
    hpjmod            = sqrt(Jnt.TraRev(k).hpj'*Jnt.TraRev(k).hpj);
    Jnt.TraRev(k).hpj = Jnt.TraRev(k).hpj/hpjmod;
    return
end
%%
%... Line numbers of the constraint equations & Pointer of next constraints
i1      = Nline;
Nline   = Nline + 1;
%%
%... Contribution to the r.h.s. of the velocity equations is null
if Flag.Velocity == 1
    niu(i1:i1,1) = 0.0;
    return
end
%%
%... Initialize variables
i  = Jnt.TraRev(k).i;
j  = Jnt.TraRev(k).j;
%
d  = Body(i).r + Body(i).A*Jnt.TraRev(k).spPi - ...
     Body(j).r - Body(j).A*Jnt.TraRev(k).spPj;
hj = Body(j).A*Jnt.TraRev(k).hpj;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i1,1) = d'*hj-Jnt.TraRev(k).l0;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i1,j1:j2) = [ hj',  hj'*Body(i).B*Jnt.TraRev(k).spPi];
    Jac(i1:i1,j3:j4) = [-hj', -hj'*Body(j).B*Jnt.TraRev(k).spPj + ...
                                d'*Body(j).B*Jnt.TraRev(k).hpj];
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    tid = Body(i).thetad;
    tjd = Body(j).thetad;
    dd  = Body(j).rd + Body(j).B*Jnt.TraRev(k).spPj*tjd - ...
          Body(i).rd - Body(i).B*Jnt.TraRev(k).spPi*tid;
    hjd = Body(j).B*Jnt.TraRev(k).hpj*tjd;
    gamma(i1:i1,1) = d'*hj*tjd^2 - 2*hjd'*dd + ...
                     hj'*(Body(i).A*Jnt.TraRev(k).spPi*tid^2 - ...
                          Body(j).A*Jnt.TraRev(k).spPj*tjd^2);
end
%
%... Finish function Joint_TraRev
end
    