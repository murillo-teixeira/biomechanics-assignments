
function [tspan,q] = PreProcessData()
%PreProcessData
%
%Summary: This function controls reading the input file for the model and 
%         the data storage into local memory.
%
%Input:   No input specified
%
%Output:  tspan - Vector with the time step information for integrator 
%         q     - Vector with the initial (estimates) system coordinates 
%
%
%% ... Access memory
global Flag NCoordinates NCoord1 NConstraints NBodyCoordinates
global NBody Body Jnt M Minv
global tstart tstep tend
global parameters solver
global A90
%
%
%% ... Integrators and time steps available
solvers = {'ode15i','ode23tb','ode23t','ode23s',...
           'ode15s','ode113','ode23'  ,'ode45'};
solver  = solvers{parameters.ode};
%% ... Set the flags for specific joint actions
Flag.InitData     = 1;

%% ... Initialize variables and matrices
NBodyCoordinates      = 3*NBody;
NCoordinates          = NBodyCoordinates;
A90                   = [0 -1; 1 0];  % 90 degree rotation matrix
Jnt.NReaction         = 0;
%
%% ... Vector with initial positions and velocities and Mass Matrix
for i = 1:NBody
    i1             = 3*i - 2;
    i2             = i1 + 1;
    i3             = i2 + 1;
%
    q(i1:i2,1)     = Body(i).r;
    q(i3:i3,1)     = Body(i).theta;
%
    M(i1:i2,i1:i2)    = Body(i).mass*eye(2);
    M(i3:i3,i3:i3)    = Body(i).inertia;
    Minv(i1:i2,i1:i2) = 1/Body(i).mass*eye(2);
    Minv(i3:i3,i3:i3) = 1/Body(i).inertia;
%
    i1             = i1 + NCoordinates;
    i2             = i1 + 1;
    i3             = i2 + 1;
%
    q(i1:i2,1)     = Body(i).rd;
    q(i3:i3,1)     = Body(i).thetad;
end
%
%% ... Initialize data for Revolute Joints
NConstraints = 0;
for k = 1:Jnt.NRevolute 
    [~,~,~,~] = Joint_Revolute ([],[],[],[],k,[]); 
end

%... Initialize data for Translation Joints
for k = 1:Jnt.NTranslation
    [~,~,~,~] = Joint_Translation ([],[],[],[],k,[]); 
end
%
%... Initialize data for Revolute-Revolute Joints
for k = 1:Jnt.NRevRev
    [~,~,~,~] = Joint_RevRev ([],[],[],[],k,[]); 
end
%
%... Initialize data for Translation-Revolute Joints
for k = 1:Jnt.NTraRev
    [~,~,~,~] = Joint_TraRev ([],[],[],[],k,[]); 
end
%
%... Initialize data for Cam Joints
for k = 1:Jnt.NCam
    [~,~,~,~]         = Joint_Cam ([],[],[],[],k,[]);
    q(NCoordinates,1) = Jnt.Cam(k).angle;
end
%
%... Convert joint type from Ground to 3x Simple Constraints 
for k = 1:Jnt.NGround
    i = Jnt.Ground(k).i;
    for j = 1:3
        Jnt.NSimple                  = Jnt.NSimple + 1;
        Jnt.Simple(Jnt.NSimple).i    = i;
        Jnt.Simple(Jnt.NSimple).type = j;
        switch j
            case 1
                Jnt.Simple(Jnt.NSimple).z0 = Body(i).r(1,1);
            case 2
                Jnt.Simple(Jnt.NSimple).z0 = Body(i).r(2,1);
            case 3
                Jnt.Simple(Jnt.NSimple).z0 = Body(i).theta;
        end
    end
end
%
%... Initialize data for Simple Constraints
for k = 1:Jnt.NSimple
    [~,~,~,~] = Joint_Simple ([],[],[],[],k,[]);
end
%
%... Initialize data for Driver Constraints
for k = 1:Jnt.NDriver
    [~,~,~,~] = Joint_Driver ([],[],[],[],k,[]); 
end
%
%% ... Initialize working space for Points of Interest
PointsOfInterest([])
%
%% ... Initilize working space for external forces
Force([]);
%
%% ... Create the time step information for the integrator
tspan = tstart:tstep:tend;
%
% ... Auxiliary quantities
NCoord1 = NCoordinates + 1;
%
%% ... Reset Initialization data flag
Flag.InitData     = 0;
%
%% Performs a kinematic analysis for the first time step to ensure data consistency 
parameters.NRMaxIter = 50;
parameters.NRTolerance = 1e-6;

%... Position analysis
[q0, Jac] = Position_Analysis(0, q(1 : NCoordinates,1));
%
%... Velocity analysis
[qd0]    = Velocity_Analysis(0, q(1 : NCoordinates, 1),Jac);

% Updates vector with initial positions and velocities
q(:, 1) = [q0; qd0];

%
%... Setup the analysis flags
Flag.Transfer     = 1;
Flag.Position     = 1;
Flag.Jacobian     = 1;
Flag.Velocity     = 1;
Flag.Acceleration = 1;
Flag.General      = 1;
%
%% ... Finish function PreProcessData
end
