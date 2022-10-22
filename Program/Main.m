% Pre-processing of data from the Laboratory of Biomechanics of Lisbon
clear;clc;

global NBody Body Jnt Pts LabData Times;

% Reads input data for the biomechanical model
ReadInput('ProcessingFile.txt');

% Reads the static data
StaticData= ReadProcessData("../Material/Kinematics & Dynamics/trial_0001_static.tsv");

% Compute the average segment lengths
ComputeAverageLengths(StaticData);
disp("gait!")
% Reads the gait data
GaitData = ReadProcessData('../Material/Kinematics & Dynamics/trial_0013_G2.tsv', 'gait analysis');
% %%
% % computes the positions and angles of the body
% Positions = EvaluatePositions(GaitData);
% figure(1)
% plot_2d_data(GaitData, Body, 'Gait', Positions)
%%

RunningData = ReadProcessData('../Material/Kinematics & Dynamics/trial_0010_Run.tsv', 'running analysis');

% computes the positions and angles of the body
Positions = EvaluatePositions(RunningData);
figure(1)
plot_2d_data(RunningData, Body, 'Running', Positions)
%%

%%
% Evaluates the drivers
EvaluateDrivers(Positions);

% Updates the data in the files to be read by the kinematic analysis
WritesModelInput('BiomechanicalModel.txt');