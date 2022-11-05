function  ReadInput(FileName)
% Write the purpose of this function here % 
global NBody Body Jnt Pts Frc
% Read the input file
H = dlmread(FileName);

% ... Initialize data
Nline = 1;

% ... Store data in Local Variables
NBody               = H(Nline, 1);
Jnt.NRevolute       = H(Nline, 2);
Jnt.NTranslation    = H(Nline, 3);
Jnt.NRevRev         = H(Nline, 4);
Jnt.NTraRev         = H(Nline, 5);
Jnt.NCam            = H(Nline, 6);
Jnt.NGround         = H(Nline, 7);
Jnt.NSimple         = H(Nline, 8);
Jnt.NDriver         = H(Nline, 9);
Pts.NPointsInt      = H(Nline, 10);
Frc.NForceAppl 	    = H(Nline, 11);
Frc.NSprDamper		= H(Nline, 12);
Frc.NVarForceAppl	= H(Nline, 13);

% ... Store initial positions for Rigid Bodies
for i = 1 : NBody
    Nline        = Nline + 1;
    Body(i).pi   = H(Nline, 1);
    Body(i).pj   = H(Nline, 2);
    Body(i).PCoM = H(Nline, 3);
    Body(i).mass = H(Nline, 4);
    Body(i).rg	 = H(Nline, 5);
end
%... Store information for Revolute Joints 
for k = 1 : Jnt.NRevolute 
    Nline               = Nline + 1; 
    Jnt.Revolute(k).i   = H(Nline, 1); 
    Jnt.Revolute(k).j   = H(Nline, 2); 
    Jnt.Revolute(k).spi = H(Nline, 3:4)'; 
    Jnt.Revolute(k).spj = H(Nline, 5:6)';
end 

% ... Store information for Driver Constraints
for k = 1 : Jnt.NDriver
    Nline                   = Nline + 1;
    Jnt.Driver(k).type      = H(Nline, 1);
    Jnt.Driver(k).i         = H(Nline, 2);
    Jnt.Driver(k).coortype  = H(Nline, 3);
    Jnt.Driver(k).j         = H(Nline, 4);
%     if(Jnt.Driver(k).type ~= 3 && ...
%             Jnt.Driver(k).type ~= 4 && ...
%             Jnt.Driver(k).type ~= 5)
%         disp('Type of driver not implemented');
%     else 
    Jnt.Driver(k).spPi      = H(Nline, 5:6)';
    Jnt.Driver(k).spPj      = H(Nline, 7:8)';
    Jnt.Driver(k).order     = H(Nline, 9); % order of spline 
    Jnt.Driver(k).Filename  = H(Nline, 10);
%     end
end

%% ...Store information for force elements
% Stores the data for the force plates
for k = 1 : Frc.NVarForceAppl
	Nline	= Nline + 1;
	Frc.VarForceAppl(k).i		 = H(Nline, 1);
	Frc.VarForceAppl(k).order 	 = H(Nline, 2); % order of spline
	Frc.VarForceAppl(k).Filename = H(Nline, 3);
end

end

