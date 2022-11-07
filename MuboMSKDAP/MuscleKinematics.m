function MuscleKinematics(q, t)
%
%Summary: This function computes the muscle kinematics (length, velocity of
%         contraction, and acceleration) for a given motion.
%
%Input:   q     - Matrix containing the positions throughout time.
%         t     - Vector of time steps.
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%
% Access global variables
global Flag NBodies Body Jnt parameters

%... Set the analysis flags
OldFlag = Flag;
Flag.Transfer = 1;
Flag.General  = 1;
Flag.Position = 1;
Flag.Jacobian = 1;
Flag.Jacobian = 0;
Flag.Velocity = 0;
Flag.Acceleration = 0;

% ... Initializes the length splines at 0 to compute the actual muscle
% lengths using the muscle contraint function
for i = 1 : Jnt.NMuscles
    Jnt.Muscle(i).spline = spline(0:0.01:10, zeros(length(0:0.01:10), 1));
end

% ... Defines number of steps
NSteps = length(t);

% Updates the parameters span
parameters.muscspan = parameters.drivspan(end) + 1 : parameters.drivspan(end) + Jnt.NMuscles;

% Goes through all steps
for i = 1 : NSteps
    
    % Computes the vector of kinematic constraints
    [ Phi, ~, ~, ~] = KinemEval(t(i),q(:,i), zeros(size(q(:,i))));
    
    % Goes through all muscles and updates their data
    for j = 1 : Jnt.NMuscles
        if (i == 1)
            Jnt.Muscle(j).Length = zeros(NSteps, 1);
        end
        Jnt.Muscle(j).Length(i) = Phi(parameters.muscspan(j), 1);
    end

    % End of the loop that goes through all steps
end

% ... Goes through all muscles, computes a spline using the muscle lengths,
% and computes the derivative of the spline to obtain velocity and
% acceleratoin of contraction
for i = 1 : Jnt.NMuscles
%     Jnt.Muscle(i).spline = spline(t, Jnt.Muscle(i).Length);
%     Jnt.Muscle(i).splined = fnder(Jnt.Muscle(i).spline, 1);
%     Jnt.Muscle(i).splinedd = fnder(Jnt.Muscle(i).spline, 2);    
    Jnt.Muscle(i).spline = DSM_spline(t, Jnt.Muscle(i).Length, 4);
    Jnt.Muscle(i).splined = ppdiff(Jnt.Muscle(i).spline, 1);
    Jnt.Muscle(i).splinedd = ppdiff(Jnt.Muscle(i).splined, 1);    
end

%... Resets the flag to what it was
Flag = OldFlag;

% End of function
end