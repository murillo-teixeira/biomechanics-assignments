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
global H NConstraints
global Flag Nline Body Jnt
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.Simple(k).i    = H(Nline,1);
    Jnt.Simple(k).type = H(Nline,2);
    Jnt.Simple(k).z0   = H(Nline,3);
    return
end
%%
%... Initialize data for this joint
if Flag.InitData == 1
    NConstraints      = NConstraints + 1;
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
i    = Jnt.Simple(k).i;
type = Jnt.Simple(k).type;
%%
%... Assemble position constraint equations
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
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*(i-1) + type;
    Jac(i1:i1,j1:j1) = 1.0;
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i1,1) = 0.0;
end
%
%... Finish function Joint_Simple
end
    