function TransformMuscleData(RefBody, RefMuscleData)
%
%Summary: This function transforms the data (scales and translates, if
%         necessary, w.r.t. the local reference frame) from the
%         biomechanical model of Stefanie Brandle to the current model.
%
%Input:   RefBody      - Structure containing properties of the bodies in
%                      the biomechanical model of Stefanie Brandle.
%         RefMuscles   - Structure containing muscle data
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%
% Access global variables
global Jnt Body
%
%% Reads the data regarding order of the bodies
[Filename] = uigetfile('*.txt','Select the MSK calibration file');

% ... Reads the data
H = readcell(Filename);

% Identifies which joint drivers should be zero due to the muscle action
MSKDriverIdentification(H);

%% Processes the data for the right and left lower limbs
% ... Defines the number of muscles
Jnt.NMuscles = max(size(RefMuscleData));

% ... Goes through all muscles and processes the data
for i = 1 : Jnt.NMuscles
    
    % Defines the name of the muscle
    Jnt.Muscle(i).name                  = ['R_', RefMuscleData(i).name];
    Jnt.Muscle(i + Jnt.NMuscles).name   = ['L_', RefMuscleData(i).name];
    
    % Defines the muscle properties, which will be later updated
    Jnt.Muscle(i).F0                    = RefMuscleData(i).F0;
    Jnt.Muscle(i + Jnt.NMuscles).F0     = RefMuscleData(i).F0;
    Jnt.Muscle(i).L0                    = RefMuscleData(i).L0;
    Jnt.Muscle(i + Jnt.NMuscles).L0     = RefMuscleData(i).L0;
    Jnt.Muscle(i).Ls                    = RefMuscleData(i).Ls;
    Jnt.Muscle(i + Jnt.NMuscles).Ls     = RefMuscleData(i).Ls;
    Jnt.Muscle(i).pen                   = RefMuscleData(i).pen;
    Jnt.Muscle(i + Jnt.NMuscles).pen    = RefMuscleData(i).pen;
    
    % Number of attachment points
    NAttach = length(RefMuscleData(i).AttachBodies);
    
    % Goes through all attachment points and processes the data
    Jnt.Muscle(i).spP                   = RefMuscleData(i).AttachP;
    Jnt.Muscle(i + Jnt.NMuscles).spP    = RefMuscleData(i).AttachP;
    Jnt.Muscle(i).spBody                = RefMuscleData(i).AttachBodies;
    Jnt.Muscle(i + Jnt.NMuscles).spBody = RefMuscleData(i).AttachBodies;
    
    for j = 1 : NAttach
        
        % Transformed coordinates. Note: the ksi of the biomechanical model
        % corresponds to -y of the model of Stefanie Brandle; The eta axis
        % corresponds to x.
        NormCoord = (RefMuscleData(i).AttachP(j,:)' - RefBody(RefMuscleData(i).AttachBodies(j)).CoM) /...
            RefBody(RefMuscleData(i).AttachBodies(j)).length;
        
        % Identifies the bodies from the current model
        if (strcmpi(RefBody(RefMuscleData(i).AttachBodies(j)).name, 'foot'))
            RBod = find(strcmpi(H(:,1), 'RFoo'));
            LBod = find(strcmpi(H(:,1), 'LFoo'));
        elseif (strcmpi(RefBody(RefMuscleData(i).AttachBodies(j)).name, 'leg'))
            RBod = find(strcmpi(H(:,1), 'RLeg'));
            LBod = find(strcmpi(H(:,1), 'LLeg'));
        elseif (strcmpi(RefBody(RefMuscleData(i).AttachBodies(j)).name, 'thigh'))
            RBod = find(strcmpi(H(:,1), 'RThi'));
            LBod = find(strcmpi(H(:,1), 'LThi'));
        elseif (strcmpi(RefBody(RefMuscleData(i).AttachBodies(j)).name, 'torso'))
            RBod = find(strcmpi(H(:,1), 'Torso'));
            LBod = find(strcmpi(H(:,1), 'Torso'));
        else
            disp('Error in TransformMuscleData: Body not found');
            pause
        end
        
        % Updates the output
        Jnt.Muscle(i).spP(j,:) = RefBody(RefMuscleData(i).AttachBodies(j)).A' * (NormCoord * H{RBod, 3});
        %Jnt.Muscle(i).spP(j,:) = Body(RBod).A' * (NormCoord * H{RBod, 3});
        Jnt.Muscle(i).spBody(j) = RBod;
        Jnt.Muscle(i + Jnt.NMuscles).spP(j,:) = RefBody(RefMuscleData(i).AttachBodies(j)).A' * (NormCoord * H{LBod, 3});
        %Jnt.Muscle(i + Jnt.NMuscles).spP(j,:) = Body(LBod).A' * (NormCoord * H{LBod, 3});
        Jnt.Muscle(i + Jnt.NMuscles).spBody(j) = LBod;
        
        % End of the loop that goes through all attachment points
    end    
    
    % End of the loop that goes through all muscles
end
%
%% 
% ... Refdefines the number of muscles as two times the original number due
%     to the definition of muscles for the two legs
Jnt.NMuscles = 2 * Jnt.NMuscles;

% End of function
end