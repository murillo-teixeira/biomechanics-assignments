clear;clc;
% Pre-processing of data from the Laboratory of Biomechanics of Lisbon

% Reads input data for the biomechanical model
ReadInput('ProcessingFile.txt');

% Reads the static data
StaticData = ReadProcessData('../Material/Kinematics & Dynamics/trial_0001_static.tsv');

% Compute the average segment lengths
ComputeAverageLengths(StaticData);

% Reads the gait data
GaitData = ReadProcessData('../Material/TDP/GaitGrid.txt');

% computes the positions and angles of the body
EvaluatePositions(GaitData);

% Evaluates the drivers
EvaluateDrivers(GaitData);

% Updates the data in the files to be read by the kinematic analysis
WritesModelInput('BiomechanicalModel.txt');