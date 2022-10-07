
function [q] = PreProcessData()
%ReadInputData
%
%Summary: This function controls reading the input file for the model and 
%         the data storage into local memory.
%
%Input:   No input specified
%
%Output:  q   - Vector with the initial (estimates) system coordinates 
%
%
%%
%... Access memory
global Flag NCoordinates NConstraints NBodyCoordinates
global NBody Body Jnt
global parameters
global A90
%
%%
%... Set the flags for specific joint actions
Flag.InitData     = 1;
%
%%
%... Build the vector with initial positions (estimates)
for i = 1:NBody
    i1 = 3*i - 2;
    i2 = i1 + 1;
    i3 = i2 + 1;
%
    q(i1:i2,1) = Body(i).r;
    q(i3:i3,1) = Body(i).theta;
end
%
%%
%... Initialize variables and matrices
NBodyCoordinates      = 3*NBody;
NCoordinates          = NBodyCoordinates;
A90                   = [0 -1; 1 0];  % 90 degree rotation matrix
parameters.NRMaxIter1 = parameters.NRMaxIter + 1;
%
%%
%... Initialize data for Revolute Joints
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
%... Reset Initialization data flag
Flag.InitData     = 0;
%
%%
%... Finish function PreProcessData
end
