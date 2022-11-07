function [Phi, Jac, niu, gamma] = Joint_Muscle(Phi, Jac, niu, gamma, k, time)
%Joint_Muscle
%
%Summary: This function controls the construction of all vectors and
%         matrices required to build the kinematic equations for a
%         muscle driver.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%         k       - Number of the muscle
%         time    - Time instant in which the positions are evaluated
%
%Output:  Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - r.h.s of the velocity equations
%         gamma   - r.h.s. of the acceleration equations
%
%Global:  Nline   - Number of the line for the next constraint equation
%         Body    - Body information
%         Jnt.Muscle - Muscle information
%
%% ... Access global memory
global NConstraints NCoordinates NJoint lambda
global Flag Nline Body Jnt
%
%% ... Initialize data for the muscle
if (Flag.InitData == 1)
    NConstraints      = NConstraints + 1;
end

%% ... Initalize the number of segments to process
NSegments = length(Jnt.Muscle(k).spBody) - 1;

%% ... Calculate the constraint position equations
if (Flag.Position == 1)
    Phi(Nline, 1) = 0;
    %
    for m = 1 : NSegments
        
        % Define local variables
        i    = Jnt.Muscle(k).spBody(m);
        spPi = Jnt.Muscle(k).spP(m,:)';
        j    = Jnt.Muscle(k).spBody(m+1);
        spPj = Jnt.Muscle(k).spP(m+1, :)';
        
        % Vector d
        d = Body(j).r + Body(j).A * spPj - (Body(i).r + Body(i).A * spPi);
        
        % Updates the constraints
        Phi(Nline, 1) = Phi(Nline, 1) + (d' * d)^(1/2);
    end
    %
    Phi(Nline, 1) = Phi(Nline, 1) - ppval(Jnt.Muscle(k).spline, time);
end
%
%% ... Calculate Jacobian matrix
if (Flag.Jacobian == 1)
    Jac(Nline, :) = zeros(1, NCoordinates);
    %
    for m = 1 : NSegments
        
        % Define local variables
        i    = Jnt.Muscle(k).spBody(m);
        spPi = Jnt.Muscle(k).spP(m,:)';
        j    = Jnt.Muscle(k).spBody(m+1);
        spPj = Jnt.Muscle(k).spP(m+1, :)';
        
        % Vector d
        d = Body(j).r + Body(j).A * spPj - (Body(i).r + Body(i).A * spPi);
        
        % Indices of body i coordinates
        c1 = 3 * (i - 1) + 1;
        c2 = c1 + 2;
        Jac(Nline, c1 : c2) = Jac(Nline, c1 : c2) +...
            [-(d' * d)^(-1/2) * d', -(d' * d)^(-1/2) * d' * Body(i).B * spPi];
        
        % Indices of body j coordinates
        c1 = 3 * (j - 1) + 1;
        c2 = c1 + 2;
        Jac(Nline, c1 : c2) = Jac(Nline, c1 : c2) +...
            [(d' * d)^(-1/2) * d', (d' * d)^(-1/2) * d' * Body(j).B * spPj];
        
        % End of the loop that goes through all muscle segments
    end
end
%
%% ... Calculate the rhs of the velocity equations
if (Flag.Velocity == 1)
    niu(Nline, 1) = ppval(Jnt.Muscle(k).splined, time);
end
%
%% ... Calculate the rhs of the acceleration equations
if (Flag.Acceleration == 1)
    gamma(Nline, 1) = 0;
    %
    for m = 1 : NSegments
        
        % Define local variables
        i    = Jnt.Muscle(k).spBody(m);
        spPi = Jnt.Muscle(k).spP(m,:)';
        j    = Jnt.Muscle(k).spBody(m+1);
        spPj = Jnt.Muscle(k).spP(m+1, :)';
        
        % Vector d
        d = Body(j).r + Body(j).A * spPj - (Body(i).r + Body(i).A * spPi);
        
        % Vector dd
        dd = Body(j).rd + Body(j).B * spPj * Body(j).thetad -...
            Body(i).rd - Body(i).B * spPi * Body(i).thetad;
        
        % Updates gamma
        gamma(Nline, 1) = gamma(Nline, 1) + (d' * d)^(-3/2) * (d' * dd)^2 -...
            (d' * d)^(-1/2) * (dd' * dd);
        
        % End of the loop that goes through all muscle segments
    end
    %
    gamma(Nline, 1) = gamma(Nline, 1) + ppval(Jnt.Muscle(k).splinedd, time);
end
%
%% ... Updates the line for the next constraint
Nline = Nline + 1;

% End of function
end