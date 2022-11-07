function MuscleCalibration(InputFile)
%
%Summary: This function reads data for a reference motion and computes
%         muscle parameters (L0, Ls) assuming the muscles are, on average,
%         in their optimal muscle fiber length in the given motion.
%
%Input:   Filename     - String with the filename. If it has already been
%                      asked for, there is no need to ask for it again.
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%
% Access global variables
global Flag Ntime NCoordinates
global Jnt

%% Reads original muscle data, taken from the work of Stefanie Brandle
[RefBody, RefMuscles] = ReferenceMuscleData();

%% Performs a kinematic analysis for the input file
% ... Changes the working directory to the static data folder
curp = cd;
cd([cd, '\StaticData']);
addpath(curp);

% ... Read the model input data and analysis profile
ReadInputData('MuscleCalibration', 1);
%
% ... Pre-process the input data
[tspan,y_init] = PreProcessData();
%
% ... Allocates memory for the output
q  = zeros(NCoordinates, Ntime);
q0 = y_init(1 : NCoordinates);
%
% ... Goes through all frames
for i = 1 : Ntime
    
    %... Position analysis
    [q(:, i), ~] = Position_Analysis(tspan(i), q0);
    
    % Updates q0
    q0 = q(:,i);
% End of the loop that goes through all frames
end
%
% PlotModel(q);
%
% ... Returns the working directory to what it was
cd(curp);
%
%% Transforms muscle data (coordinate transformation and scaling) from the model of Stefanie Brandle to the current model
TransformMuscleData(RefBody, RefMuscles);
%
%% Computes muscle kinematics
MuscleKinematics(q, tspan);
%
%% Rescales muscle parameters according to the current biomechanical model
for i = 1 : Jnt.NMuscles
    
    % Average length
    lMT = mean(Jnt.Muscle(i).Length);
    
    % Ratio between the original L0 and Ls
    beta = Jnt.Muscle(i).L0 / Jnt.Muscle(i).Ls;
    
    % Novel L0 and Ls
    Jnt.Muscle(i).Ls = lMT / (beta * cos(Jnt.Muscle(i).pen) + 1);
    Jnt.Muscle(i).L0 = beta * Jnt.Muscle(i).Ls;
    % Maximum isometric muscle force will also be redefined because the
    % original forces seem too low. They were computed assuming a specific
    % muscle strength of 31.39 N/cm^2. Instead of this value, i will
    % consider 45 N/cm^2, which is considered in the Lisbon Lower Limb
    % Muscle Model. For forces too large, i will not change the value.
    if (Jnt.Muscle(i).F0 < 2000)
        Jnt.Muscle(i).F0 = 45 * (Jnt.Muscle(i).F0 / 31.39);
    end
    % Defines the maximum velocity as 10 Ls
    Jnt.Muscle(i).Vmax = 10 * Jnt.Muscle(i).L0;
    % End of the loop that goes through all muscles
end
%
%% ... Resets the flag structure
%... Initialize data
Flag.ReadInput    = 0;
Flag.InitData     = 0;
Flag.Transfer     = 0;
Flag.Position     = 0;
Flag.Jacobian     = 0;
Flag.Velocity     = 0;
Flag.Acceleration = 0;
Flag.Reaction     = 0;
Flag.General      = 0;

% End of function
end