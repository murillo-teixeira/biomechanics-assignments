function MSKDriverIdentification(H)
%
%Summary: This function identifies the drivers that connect bodies that are
%         going to be actuated by muscles
%
%Input:   H      - Matrix with the data from the MSK file
%
%Output:  No output specified. However, data transfered over the global
%         memory for the program with the following structure.
%
% Access global variables
global Jnt Body MSKDrivers

% ... Initializes a vector to contain the driver joints that are actuacted
% by muscles
MSKDrivers = [];

% ... Goes through all muscles and processes the data
for i = 1 : Jnt.NDriver
    
    % Processes body i
    if (Jnt.Driver(i).i > 0)
        % Find the position of the current body in H
        BodyPosi = find([H{:,2}] == Jnt.Driver(i).i);
        
        if (strcmpi(H{BodyPosi,1}, 'RFoo') || strcmpi(H{BodyPosi,1}, 'LFoo') ||...
                strcmpi(H{BodyPosi,1}, 'RLeg') || strcmpi(H{BodyPosi,1}, 'LLeg') ||...
                strcmpi(H{BodyPosi,1}, 'RThi') || strcmpi(H{BodyPosi,1}, 'LThi') ||...
                strcmpi(H{BodyPosi,1}, 'Torso'))
            Bodyi = 1;
        else
            Bodyi = 0;
        end
    else
        Bodyi = 0;
    end
       
    % Processes body j
    if (Jnt.Driver(i).j > 0)
        % Find the position of the current body in H
        BodyPosj = find([H{:,2}] == Jnt.Driver(i).j);
        
        if (strcmpi(H{BodyPosj,1}, 'RFoo') || strcmpi(H{BodyPosj,1}, 'LFoo') ||...
                strcmpi(H{BodyPosj,1}, 'RLeg') || strcmpi(H{BodyPosj,1}, 'LLeg') ||...
                strcmpi(H{BodyPosj,1}, 'RThi') || strcmpi(H{BodyPosj,1}, 'LThi') ||...
                strcmpi(H{BodyPosj,1}, 'Torso'))
            Bodyj = 1;
        else
            Bodyj = 0;
        end
    else
        Bodyj = 0;
    end
    
    if (Bodyi == 1 && Bodyj == 1)
        MSKDrivers = [MSKDrivers; i];
    end    
end

% End of function
end