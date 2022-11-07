function [Phi,Jac,niu,gamma] = Joint_Simple (Phi,Jac,niu,gamma,k,time)
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
%         k       - Number of the Simple joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Simple - Simple joint information
%
%
%%
%... Access global memory
global H NConstraints lambda
global Flag Nline Body Jnt NJoint
%
%% ... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.Simple(k).i    = H(Nline,1);
    Jnt.Simple(k).type = H(Nline,2);
    Jnt.Simple(k).z0   = H(Nline,3);
    return
end
%
%%... Initialize data for this joint
if Flag.InitData == 1
    NConstraints      = NConstraints + 1;
%
%... Prepare Joint Reactions Report
    NJoint                      = Jnt.NReaction + 1;
    Jnt.NReaction               = NJoint;
    Jnt.Reaction(NJoint).Number = k;
    Jnt.Reaction(NJoint).i      = Jnt.Simple(k).i;
    Jnt.Reaction(NJoint).j      = 0;
    switch Jnt.Simple(k).type
        case 1
            Jnt.Reaction(NJoint).Type   = '.......Simple_X';
        case 2
            Jnt.Reaction(NJoint).Type   = '.......Simple_Y';
        case 3
            Jnt.Reaction(NJoint).Type   = '.......Simple_O';
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
    i    = Jnt.Simple(k).i;
    type = Jnt.Simple(k).type;
end
%
%% ... Assemble position constraint equations
if Flag.Position == 1 
    switch type
        case 1
            aux = Body(i).r(1,1) - Jnt.Simple(k).z0;
        case 2
            aux = Body(i).r(2,1) - Jnt.Simple(k).z0;
        case 3
            aux = Body(i).theta  - Jnt.Simple(k).z0;
    end
    Phi(i1:i1,1) = aux;
end
%
%% ... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*(i-1) + type;
    Jac(i1:i1,j1:j1) = 1.0;
end
%
%% ... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i1,1) = 0.0;
end
%
%
%% ... Evaluate the Joint Reaction Forces for Report
if Flag.Reaction == 1
    NJoint                  = NJoint +1;
    j1                      = 3*i-2;
    j2                      = j1 + 2;
    Jnt.Reaction(NJoint).gi = -Jac(i1:i1,j1:j2)'*lambda(i1:i1,1);
end
%
%% ... Finish function Joint_Revolute
end
    