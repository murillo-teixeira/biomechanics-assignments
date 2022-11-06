function [Filename] = ReadInputData()
%
%Summary: This function controls reading the input file for the model and 
%         the data storage into local memory.
%
%Input:   No input specified
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%         NBody        - Number of Rigid Bodies in the system
%         NRevolute    - Number of Revolute Joints
%         NTranslation - Number of Translation Joints
%         NRevRev      - Number of Revolute-Revolute Joints
%         NTransRev    - Number of Translation-Revolute Joints
%         NCam         - Number of Cam Joints
%         NSimple      - Number of Simple Constraints
%         NDriver      - Number of drivers
%         NPtInterest  - Number of points of interest
%
% Jorge Ambrosio
% Version 1.0     May 12, 2020
%
%% ... Access memory
global H Flag Nline Ntime
global NBody Body Jnt Pts Frc gravity
global parameters
global tstart tend tstep trep
%
%% ... Query for the input file with the model
[Filename] = uigetfile('*.txt','Select the Model Data file');
%
%... Read data from input file
H = dlmread(Filename); %datamatrix 
%
%... Initialize data
Nline             = 1;
Flag.ReadInput    = 1;
Flag.InitData     = 0;
Flag.Transfer     = 0;
Flag.Position     = 0;
Flag.Jacobian     = 0;
Flag.Velocity     = 0;
Flag.Acceleration = 0;
Flag.Reaction     = 0;
Flag.General      = 0;
%
%% ... Store data in Local Variables
NBody             = H(Nline,1);
Jnt.NRevolute     = H(Nline,2);
Jnt.NTranslation  = H(Nline,3);
Jnt.NRevRev       = H(Nline,4);
Jnt.NTraRev       = H(Nline,5);
Jnt.NCam          = H(Nline,6); 
Jnt.NGround       = H(Nline,7);
Jnt.NSimple       = H(Nline,8);
Jnt.NDriver       = H(Nline,9);
Pts.NPointsInt    = H(Nline,10);
Frc.NForceAppl    = H(Nline,11);
Frc.NSprDamper    = H(Nline,12);
Frc.NVarForceAppl = H(Nline, 13);
%
%% ... Store initial positions for Rigid Bodies
for i = 1:NBody
    BodyData(i,[],[]);
end
%
%% ... Store initial positions kinematic joints
%
%... Store information for Revolute Joints 
for k = 1:Jnt.NRevolute 
    [~,~,~,~] = Joint_Revolute ([],[],[],[],k,[]); 
end

%... Store information for Translation Joints
for k = 1:Jnt.NTranslation
    [~,~,~,~] = Joint_Translation ([],[],[],[],k,[]); 
end
%
%... Store information for Revolute-Revolute Joints
for k = 1:Jnt.NRevRev
    [~,~,~,~] = Joint_RevRev ([],[],[],[],k,[]); 
end
%
%... Store information for Translation-Revolute Joints
for k = 1:Jnt.NTraRev
    [~,~,~,~] = Joint_TraRev ([],[],[],[],k,[]); 
end
%
%... Store information for Cam Joints
for k = 1:Jnt.NCam
    [~,~,~,~] = Joint_Cam ([],[],[],[],k,[]); 
end
%
%... Store information for Ground Joints 
for k = 1:Jnt.NGround
    Nline           = Nline + 1;
    Jnt.Ground(k).i = H(Nline,1);
end
%
%... Store information for Simple Constraints
for k = 1:Jnt.NSimple
    [~,~,~,~] = Joint_Simple ([],[],[],[],k,[]);
end
%
%... Store information for Driver Constraints
for k = 1:Jnt.NDriver
    [~,~,~,~] = Joint_Driver ([],[],[],[],k,[]); 
end
%
%% ...Store information for Points of Interest 
PointsOfInterest([])
%
%% ...Store information for force elements
%
%... Store information for external applied forces
[~] = ForceApplied([]); 
%
%... Store information for external applied forces
[~] = ForceSprDamper([]); 

%... Store information for applied forces changing with time
[~] = VarForceApplied([]);

%% ... Store the gravity acceleration vector
Nline   = Nline + 1;  
gravity = H(Nline,1:2)';
%
%% ... Store the Baumgarte stabilization parameters
Nline                      = Nline + 1;  
parameters.ode             = H(Nline,1);
parameters.MotionEqSolver  = H(Nline,2); 
parameters.alpha           = H(Nline,3);
parameters.beta            = H(Nline,4)^2; 
%
%% ...Store time analysis information 
Nline  = Nline + 1; 
tstart = H(Nline,1);
tstep  = H(Nline,2);
tend   = H(Nline,3);
trep   = H(Nline,4);
%
Ntime  = floor((tend-tstart)/tstep)+1;
%
%% ... Reset Read Input Data Flag
Flag.ReadInput = 0;
%
%%
%... Finish function ReadInputData
end
