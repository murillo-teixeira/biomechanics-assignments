function [Phi,Jac,niu,gamma] = Joint_Translation (Phi,Jac,niu,gamma,k,time)
%Joint_Translation
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a
%         translation joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the Translation joint
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Translation - Translation joint information
%
%
%% ... Access global memory
global H NConstraints A90 lambda
global Flag Nline Body Jnt NJoint
%
%% ... Read input for this joint
if Flag.ReadInput == 1
    Nline                   = Nline + 1;
    Jnt.Translation(k).i    = H(Nline,1);
    Jnt.Translation(k).j    = H(Nline,2);
    Jnt.Translation(k).spPi = H(Nline,3:4)';
    Jnt.Translation(k).spQi = H(Nline,5:6)';
    Jnt.Translation(k).spPj = H(Nline,7:8)';
    Jnt.Translation(k).spQj = H(Nline,9:10)';
    return
end
%
%% ... Initialize data for this joint
if Flag.InitData == 1
    NConstraints           = NConstraints + 2;
    Jnt.Translation(k).spi = Jnt.Translation(k).spPi - ...
                             Jnt.Translation(k).spQi;
    Jnt.Translation(k).hpj = A90*(Jnt.Translation(k).spPj - ...
                                  Jnt.Translation(k).spQj);
%
%... Prepare Joint Reactions Report
    NJoint                      = Jnt.NReaction + 1;
    Jnt.NReaction               = NJoint;
    Jnt.Reaction(NJoint).Number = k;
    Jnt.Reaction(NJoint).i      = Jnt.Translation(k).i;
    Jnt.Reaction(NJoint).j      = Jnt.Translation(k).j;
    Jnt.Reaction(NJoint).Type   = '....Translation';
    return
end
%
%% ... Line numbers of constraint equations & Pointer of next constraints
if Flag.General == 1
    i1    = Nline;
    i2    = i1 + 1;
    Nline = Nline + 2;
%
%... Initialize variables
    i   = Jnt.Translation(k).i;
    j   = Jnt.Translation(k).j;
%
    si  = Body(i).A*Jnt.Translation(k).spi;
    hj  = Body(j).A*Jnt.Translation(k).hpj;
    sPi = Body(i).A*Jnt.Translation(k).spPi;
    sPj = Body(j).A*Jnt.Translation(k).spPj;
    sij = Body(i).r + sPi - Body(j).r - sPj;
end
%
%% ... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i2,1) = [hj'*si; ...
                    hj'*sij];
    if (Flag.Jacobian == 0 && Flag.Acceleration == 0); return; end
end
%
%% ... Initialize variables
Bspi  = Body(i).B*Jnt.Translation(k).spi;
Bhpj  = Body(j).B*Jnt.Translation(k).hpj;
BspPi = Body(i).B*Jnt.Translation(k).spPi;
BspPj = Body(j).B*Jnt.Translation(k).spPj;
%
%% ... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
%
    Jac(i1:i2,j1:j2) = [ zeros(1,2), hj'*Bspi; ...
                           hj'     , hj'*BspPi];
    Jac(i1:i2,j3:j4) = [ zeros(1,2), si'*Bhpj;...
                          -hj'     , sij'*Bhpj-hj'*BspPj];
end
%
%% ... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    tid  = Body(i).thetad;
    tjd  = Body(j).thetad;
    sid  = Body(i).B*Jnt.Translation(k).spi*tid;
    hjd  = Bhpj*tjd;
    sijd = Body(i).rd + BspPi*tid - Body(j).rd - BspPj*tjd;
%
    gamma(i1:i2,1) = [(-sid'*Bhpj*tjd  + si'*hj*tjd^2 - ...
                       hjd'*Bspi*tid  + hj'*si*tid^2); ...
                      (-sijd'*Bhpj*tjd + sij'*hj*tjd^2 -...
                       hjd'*sijd      + hj'*(sPi*tid^2-sPj*tjd^2))];
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
    