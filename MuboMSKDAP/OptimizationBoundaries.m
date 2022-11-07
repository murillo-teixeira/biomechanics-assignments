function [lb, ub, optmusc] = OptimizationBoundaries(time)
%PreProcessData
%
%Summary: This function defines the boundary conditions for the
%         optimization problem.
%
%Input:   time     - Current time step.
%
%Output:  lb       - Vector of lower bounds
%         ub       - Vector of upper bounds
%         scalJac  - Vector of scaling factors to scale the Jacobian
%         matrix. This is mainly because of the transformation of the
%         Lagrangian multiplier of muscles into activations.
%
%% ... Access memory
global NConstraints Jnt parameters

%% ... Initalizes lb and ub without any limits
lb = -Inf * ones(NConstraints, 1);
ub = Inf * ones(NConstraints, 1);

%% ... Processes the muscle entries to be bounded between 0 and 1 (if a 
%      hill muscle is used) or Inf (if no Hill model is used)
%
% Allocates memory for the output
optmusc.scalJac = ones(NConstraints, 1);
optmusc.fpe = zeros(Jnt.NMuscles, 1);
%
% Goes through all muscles and computes the Hill components of the
% contractile element
for i = 1 : Jnt.NMuscles
    
    % Current musculo-tendon length and velocity of contraction
    lMT = ppval(Jnt.Muscle(i).spline, time);
    lMTd = ppval(Jnt.Muscle(i).splined, time);
    
    % Computes current pennation angle
    PenAngle = atan((Jnt.Muscle(i).L0 * sin(Jnt.Muscle(i).pen)) /...
        (lMT - Jnt.Muscle(i).Ls));
    
    % Current muscle length and velocity of contraction
    lM = (lMT - Jnt.Muscle(i).Ls) / cos(PenAngle);
    lMd = lMTd * cos(PenAngle);
    
    % Computes the force-length and force-velocity relationships
    Fl = HillForceLength(lM, Jnt.Muscle(i).F0, Jnt.Muscle(i).L0);
    Fld = HillForceVelocity(lMd, Jnt.Muscle(i).F0, Jnt.Muscle(i).Vmax);
    
    % computes the force from the passive element
    Fpe = HillPassiveForce(lM, Jnt.Muscle(i).F0, Jnt.Muscle(i).L0);
    
    % Computes the scaling parameter for the Jacobian matrix to make the
    % lagrange multiplier muscle activation
    optmusc.scalJac(parameters.muscspan(i)) = (Fl * Fld / Jnt.Muscle(i).F0) * cos(PenAngle);
    if (parameters.passelem == 1)
        optmusc.fpe(i) = Fpe;
    end
    Jnt.Muscle(i).Param = [Fl, Fld, lM, lMd, Fpe];

    % Updates the lower and upper bounds of the muscle entries
    lb(parameters.muscspan(i)) = 0;
    if (parameters.hill == 1)
        ub(parameters.muscspan(i)) = 1;
    end
    
    % End of the loop that goes through all muscles
end

% End of function
end