function [Phi,Jac,niu,gamma] = Joint_Revolute (Phi,Jac,niu,gamma,k,time)
%Joint_Revolute
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a revolute
%         joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the Revolute joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Revolute - Revolute joint information
%
%
%% ... Access global memory
global H NConstraints lambda
global Flag Nline Body Jnt NJoint
%
%% ... Read input for this joint
if Flag.ReadInput == 1
    Nline                = Nline + 1;
    Jnt.Revolute(k).i    = H(Nline,1);
    Jnt.Revolute(k).j    = H(Nline,2);
    Jnt.Revolute(k).spPi = H(Nline,3:4)';
    Jnt.Revolute(k).spPj = H(Nline,5:6)';
    return
end
%
%% ... Initialize data for this joint
if Flag.InitData == 1
    NConstraints = NConstraints + 2;
%
%... Prepare Joint Reactions Report
    NJoint                      = Jnt.NReaction + 1;
    Jnt.NReaction               = NJoint;
    Jnt.Reaction(NJoint).Number = k;
    Jnt.Reaction(NJoint).i      = Jnt.Revolute(k).i;
    Jnt.Reaction(NJoint).j      = Jnt.Revolute(k).j;
    Jnt.Reaction(NJoint).Type   = '.......Revolute';
    return
end
%
%% ... Line numbers of constraint equations & Pointer of next constraints
if Flag.General == 1
    i1      = Nline;
    i2      = i1 + 1;
    Nline   = Nline + 2;
%
%... Initialize variables
    i = Jnt.Revolute(k).i;
    j = Jnt.Revolute(k).j;
end
%
%% ... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i2,1) = Body(i).r + Body(i).A*Jnt.Revolute(k).spPi - ...
                   Body(j).r - Body(j).A*Jnt.Revolute(k).spPj;
end
%
%% ... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i2,j1:j2) = [ eye(2), Body(i).B*Jnt.Revolute(k).spPi];
    Jac(i1:i2,j3:j4) = [-eye(2),-Body(j).B*Jnt.Revolute(k).spPj];
end
%
%% ... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i2,1) = Body(i).A*Jnt.Revolute(k).spPi*Body(i).thetad^2 - ...
                     Body(j).A*Jnt.Revolute(k).spPj*Body(j).thetad^2;
end
%
%% ... Evaluate the Joint Reaction Forces for Report
if Flag.Reaction == 1
    NJoint                  = NJoint +1;
    Jnt.Reaction(NJoint).gi = -Jac(i1:i2,j1:j2)'*lambda(i1:i2,1);
    Jnt.Reaction(NJoint).gj = -Jac(i1:i2,j3:j4)'*lambda(i1:i2,1);
end
%
%% ... Finish function Joint_Revolute
end
    