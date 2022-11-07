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
%% ... Access global memory
global H NConstraints lambda
global Flag Nline Body Jnt NJoint
%
%% ... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.RevRev(k).i    = H(Nline,1);
    Jnt.RevRev(k).j    = H(Nline,2);
    Jnt.RevRev(k).d0   = H(Nline,3);
    Jnt.RevRev(k).spPi = H(Nline,4:5)';
    Jnt.RevRev(k).spPj = H(Nline,6:7)';
    return
end
%
%% ... Initialize data for this joint
if Flag.InitData == 1
    NConstraints = NConstraints + 1;
    Jnt.RevRev(k).l2   = Jnt.RevRev(k).d0^2;
%
%... Prepare Joint Reactions Report
    NJoint                      = Jnt.NReaction + 1;
    Jnt.NReaction               = NJoint;
    Jnt.Reaction(NJoint).Number = k;
    Jnt.Reaction(NJoint).i      = Jnt.RevRev(k).i;
    Jnt.Reaction(NJoint).j      = Jnt.RevRev(k).j;
    Jnt.Reaction(NJoint).Type   = '...Rev-Revolute';
    return
end
%
%% ... Line numbers of constraint equations & Pointer of next constraints
if Flag.General == 1
    i1      = Nline;
    Nline   = Nline + 1;
%
%... Initialize variables
    i = Jnt.RevRev(k).i;
    j = Jnt.RevRev(k).j;
%
    d = Body(j).r + Body(j).A*Jnt.RevRev(k).spPj - ...
        Body(i).r - Body(i).A*Jnt.RevRev(k).spPi;
end
%
%% ... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i1,1) = d'*d - Jnt.RevRev(k).l2;
end
%
%% ... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i1,j1:j2) = 2*[-d', -d'*Body(i).B*Jnt.RevRev(k).spPi];
    Jac(i1:i1,j3:j4) = 2*[ d',  d'*Body(j).B*Jnt.RevRev(k).spPj];
end
%
%% ... Assemble the right hand side of the Acceleration Equations 
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
%% ... Evaluate the Joint Reaction Forces for Report
if Flag.Reaction == 1
    NJoint                  = NJoint +1;
    Jnt.Reaction(NJoint).gi = -Jac(i1:i1,j1:j2)'*lambda(i1:i1,1);
    Jnt.Reaction(NJoint).gj = -Jac(i1:i1,j3:j4)'*lambda(i1:i1,1);
end
%
%% ... Finish function Joint_Revolute
end
    