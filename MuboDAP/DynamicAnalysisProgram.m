clear all
%
%DynamicAnalysisProgram
%
%Summary: This program preforms the dynamic analysis of a general planar
%         mechanism for which the model is supplied with a plain text file.
%
%Global:  solver - Time integration function selected for anaysis
%
% Jorge Ambrosio
% Version 1.0     May 12, 2020
%
%% ... Access global memory
global solver
%
%% ... Read the model input data and analysis profile
[Filename] = ReadInputData();
%
%% ... Pre-process the input data
[tspan,y_init] = PreProcessData();
%
%% ... Perform the Dynamic Analysis
[t,y] = Integration_odes(y_init,solver,tspan);
%
%% ... Post-Process the analysis results
PostProcessResults(t,y,Filename);
%
%... Terminate the Dynamic Analysis Program
