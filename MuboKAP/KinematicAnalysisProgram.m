clear all
%
%KinematicAnalysisProgram
%
%Summary: This program preforms the kinematic analysis of a general planar
%         mechanism for which the model is supplied with a plain text file.
%
%Global:  NConstraints - Number of kinematic constraints
%         NCoordinates - Number of coordinates
%
%
%%
%... Access global memory
global NConstraints NCoordinates
%
%... Read the model input data and analysis profile
Filename = ReadInputData();
%
%... Pre-process the input data
[q0] = PreProcessData();
%
%... Check if the number of coordinates and constraints is equal so that
%    the analysis can proceed
if NConstraints ~= NCoordinates
        disp('FATAL ERROR')
        string = ['# Constraints (' num2str(NConstraints) ...
                  ') differs from # Coordinates (' ...
                   num2str(NCoordinates) ')'];
        disp(string)
        disp('Program stopped')
    stop
end
%
%... Perform the Kinematic Analysis
[t,q,qd,qdd] = Kinematic_Analysis(q0);
%
%... Post-Process the analysis results
PostProcessResults(t,q,qd,qdd,Filename);
%
%... Terminate the Kinematic Analysis Program
