clear all
restoredefaultpath
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
%% ... Install the optimization solver
% Install the optimization solver
% Current path
p = cd;    
% Changes the directory to the OPTI folder
cd([p,'\OPTI']);

% Install the toolbox
opti_Install;

% Changes back to the previous directory
cd(p);
% Add the optimization solver folder to the path
addpath([cd,'\OPTI']    );

%% ... Access memory
global solver NCoordinates parameters

%% ... Menu for the dynamic analysis
disp('1 - Dynamic analysis using joint actuators (No muscles)');
disp('2 - Dynamic analysis using muscle actuators (No Hill model)');
disp('3 - Dynamic analysis using muscle actuators (Hill model)');
opt = input('Please select the analysis to be performed.\n');
%
switch opt
    case 1
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
    case {2, 3}
        %% ... Initializes the parameters variable
        if (opt == 2)
            parameters.hill = 0;
            parameters.passelem = 0;
        else
            parameters.hill = 1;
            % Defines if the passive component is considered
            parameters.passelem = 0;
        end
        %
        %% ... Read the model input data and analysis profile
        [Filename] = ReadInputData();
        
        %% ... Pre-process the input data
        [tspan,y_init] = PreProcessData();
        
        %% ... Performs a kinematic analysis to ensure consistency of the 
        %      data with the biomechanical model
        [ t,q,qd,qdd ] = Kinematic_Analysis( y_init(1 : NCoordinates, 1) );
        %
        %% ... Calibrates muscle data
        MuscleCalibration(Filename);
        %
        %% ... Read the model input data again to reset the data read 
        %      for which there were no muscles
        [Filename] = ReadInputData(Filename, 0);        
        %
        %% ... Pre-processes the input data again to make sure everything 
        %      is ready for the dynamic analysis
        [tspan,y_init] = PreProcessData();
        %
        %% ... Perform the Dynamic Analysis
        lmult = optimization_dyn(q, qd, qdd, t);
        %
        %% ... Post-Process the analysis results
        PostProcessMSKResults (t',q,qd,qdd,lmult,Filename);
        %
        %... Terminate the Dynamic Analysis Program
% End of the switch case
end
