function [ Phi,Jac,niu,gamma ] = KinemEval(time,q,qd)
%KinemEval
%
%Summary: This function controls the construction of all vectors and
%         matrices required to solve the complete kinematic analysis, i.e.,
%         the vector with the constraint equations, the Jacobian matrix,
%         the r.h.s. of the velocity and acceleration equations. 
%         Furthermore, it transfers the variables from the global arrays 
%         to local storage and builds the body matrics A and B
%
%Input:   time - Time instant in which the positions are evaluated
%         q    - System positions
%         qd   - System velocities
%
%Output:  Phi   - Vector with the kinematic constraints
%         Jac   - Jacobian matrix
%         niu   - r.h.s of the velocity equations
%         gamma - r.h.s. of the acceleration equations
%
%
%% ... Access global memory
global Flag NCoordinates NConstraints NBodyCoordinates Nline
global Body NBody Jnt
%
%% ... Initialize vectors and Jacobian matrix
Phi   = zeros(NConstraints,1);
Jac   = zeros(NConstraints,NCoordinates);
niu   = zeros(NConstraints,1);
gamma = zeros(NConstraints,1);
Nline = 1;
%
%% ... Transfer positions & velocities from global to local storage
if Flag.Transfer == 1
    for i = 1:NBody
        BodyData(i,q,qd);
    end
%
%... Transfer Cam angles from global to local storage, ensuring that
%... the angle is kept in the [0, 360º] range
    for k = 1:Jnt.NCam
        Jnt.Cam(k).angle  = wrapTo2Pi(q(NBodyCoordinates+k,1));
        Jnt.Cam(k).angled = qd(NBodyCoordinates+k,1);
    end
end
%
%% ... Contributions by Revolute Joints
for k = 1:Jnt.NRevolute
    [Phi,Jac,niu,gamma] = Joint_Revolute (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Translation Joints
for k = 1:Jnt.NTranslation
    [Phi,Jac,niu,gamma] = Joint_Translation (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Revolute-Revolute Joints
for k = 1:Jnt.NRevRev
    [Phi,Jac,niu,gamma] = Joint_RevRev (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Translation-Revolute Joints
for k = 1:Jnt.NTraRev
    [Phi,Jac,niu,gamma] = Joint_TraRev (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Simple Joints
for k = 1:Jnt.NSimple
    [Phi,Jac,niu,gamma] = Joint_Simple (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Driving constraints
for k = 1:Jnt.NDriver
    [Phi,Jac,niu,gamma] = Joint_Driver (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Contributions by Cam Joints
for k = 1:Jnt.NCam
    [Phi,Jac,niu,gamma] = Joint_Cam (Phi,Jac,niu,gamma,k,time);
end
%
%% ... Finalize function KinemEval
end


