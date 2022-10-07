function [Phi,Jac,niu,gamma] = Joint_RevRev (Phi,Jac,niu,gamma,k,time)
%Joint_RevRev
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a
%         composite revolute-revolute joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the Revolute-Revolute joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.RevRev - Revolute-Revolute joint information
%
%
%%
%... Access global memory
global H NConstraints
global Flag Nline Body Jnt
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.RevRev(k).i    = H(Nline,1);
    Jnt.RevRev(k).j    = H(Nline,2);
    Jnt.RevRev(k).d0   = H(Nline,3);
    Jnt.RevRev(k).spPi = H(Nline,4:5)';
    Jnt.RevRev(k).spPj = H(Nline,6:7)';
    return
end
%%
%... Initialize data for this joint
if Flag.InitData == 1
    NConstraints = NConstraints + 1;
    Jnt.RevRev(k).l2   = Jnt.RevRev(k).d0^2;
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
i = Jnt.RevRev(k).i;
j = Jnt.RevRev(k).j;
%
d = Body(j).r + Body(j).A*Jnt.RevRev(k).spPj - ...
    Body(i).r - Body(i).A*Jnt.RevRev(k).spPi;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i1,1) = d'*d - Jnt.RevRev(k).l2;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i1,j1:j2) = 2*[-d', -d'*Body(i).B*Jnt.RevRev(k).spPi];
    Jac(i1:i1,j3:j4) = 2*[ d',  d'*Body(j).B*Jnt.RevRev(k).spPj];
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    tid = Body(i).thetad;
    tjd = Body(j).thetad;
%
    dd  = Body(j).rd + Body(j).B*Jnt.RevRev(k).spPj*tjd - ...
          Body(i).rd - Body(i).B*Jnt.RevRev(k).spPi*tid;
    gamma(i1:i1,1) = -dd'*dd - d'*(Body(i).A*Jnt.RevRev(k).spPi*tid^2 - ...
                                   Body(j).A*Jnt.RevRev(k).spPj*tjd^2);
end
%
%... Finish function Joint_RevRev
end
    