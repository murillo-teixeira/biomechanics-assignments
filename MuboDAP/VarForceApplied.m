function [g] = VarForceApplied(g, t)
%ForceApplied
%
%Summary: This function controls all actions regarding data for the  
%         external applied forces, including its input, update and 
%         storage into local memory.
%
%Input:   g            - Vector with system forces
%
%Output:  g            - Vector with system forces
%
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access memory
global H Frc Flag Nline
global Body
%
%% ... Store external force definition data
if     Flag.ReadInput == 1
    for k = 1:Frc.NVarForceAppl
        Nline                        = Nline + 1; 
        Frc.VarForceAppl(k).i        = H(Nline, 1);
        Frc.VarForceAppl(k).order    = H(Nline, 2);
        x                            = H(Nline, 3); % File for actuator input
        Frc.VarForceAppl(k).Filename = sprintf("VarForceAppl_%03d.txt",x);        
    end
    g = [];
end
%
%% ... Initialize data for the ground reaction forces
if     Flag.InitData == 1
    
    for k = 1 : Frc.NVarForceAppl
        %
        %... Read data from input file
        H   = dlmread(Frc.VarForceAppl(k).Filename);
        %
        %... Evaluate the spline interpolation and its derivatives
        spline_type             = 1;
        order                   = Frc.VarForceAppl(k).order;
        Frc.VarForceAppl(k).Spline   = [DSM_spline(H(:,1)',H(:,2)',order);
            DSM_spline(H(:,1)',H(:,3)',order);
            DSM_spline(H(:,1)',H(:,4)',order);
            DSM_spline(H(:,1)',H(:,5)',order);
            DSM_spline(H(:,1)',H(:,6)',order)
            ];
    end
end
%
%% ... Transfer velocities from global to local storage
if     Flag.Acceleration == 1
    
    for k = 1 : Frc.NVarForceAppl
        
        % Body in which the force is applied
        i = Frc.VarForceAppl(k).i;
        
        % Defines the force and moment to be applied
        f = [ppval(Frc.VarForceAppl(k).Spline(1), t); ppval(Frc.VarForceAppl(k).Spline(2), t)];
        n = ppval(Frc.VarForceAppl(k).Spline(3), t);
        
        % Defines the application point in the global reference frame
        rf = [ppval(Frc.VarForceAppl(k).Spline(4), t); ppval(Frc.VarForceAppl(k).Spline(5), t)];
        
        % Defines the coordinates of the application point with respect to
        % the center of mass of the body
        spPi = Body(i).A' * (rf - Body(i).r);
            
        % Application of the force in Body i
        i1 = 3 * i - 2;
        i3 = i1 + 2;
        %
        g(i1 : i3,1) = g(i1 : i3,1) + ...
            ApplyForce(f, spPi, Body(i).A);
        g(i3:i3,1) = g(i3:i3,1) + n;
    end
end
%
%% ... Finish function ForceApplied
end
