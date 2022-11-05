% Pre-processing of data from the Laboratory of Biomechanics of Lisbon
clear;clc;

global NBody Body Jnt Pts LabData Times Frc;

% Reads input data for the biomechanical model
ReadInput('ProcessingFile.txt');

% Reads the static data
StaticData= ReadProcessData("../Material/Kinematics & Dynamics/trial_0001_static.tsv", 'static');

% Compute the average segment lengths
ComputeAverageLengths(StaticData);

% Compute total body mass from ground reaction forces and update the mass
% and inertia of the bodies
ComputeBodyProperties();        % Second Part!

disp("gait!")
% Reads the gait data
GaitData = ReadProcessData('../Material/Kinematics & Dynamics/trial_0013_G2.tsv', 'gait analysis');
% %%
 % computes the positions and angles of the body
 Positions = EvaluatePositions(GaitData);
 figure(1)
 plot_2d_data(GaitData, Body, 'Gait', Positions)
%% Running Analysis 

%RunningData = ReadProcessData('../Material/Kinematics & Dynamics/trial_0010_Run.tsv', 'running analysis');

% computes the positions and angles of the body
%Positions = EvaluatePositions(RunningData);
%figure(1)
%plot_2d_data(RunningData, Body, 'Running', Positions)
%%

%%
% Evaluates the drivers
EvaluateDrivers(Positions);

% Process the ground reaction forces
ReadGRF(GaitData.Frequency);        % Second Part!

% Updates the data in the files to be read by the kinematic analysis
WritesModelInput('..\MuboKap\BiomechanicalModelRunning.txt');