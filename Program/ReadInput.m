function  ReadInput(FileName)
% Write the purpose of this function here % 
global NBody Body Jnt Pts
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

% ... Store initial positions for Rigid Bodies
for i = 1 : NBody
    Nline = Nline + 1;
    Body(i).pi   = H(Nline, 1);
    Body(i).pj   = H(Nline, 2);
    Body(i).PCoM = H(Nline, 3);
end

% ... Store information for Revolute Joints
for k = 1 : Jnt.NRevolute
    Nline                   = Nline + 1;
    Jnt.Driver(k).type      = H(Nline, 1);
    Jnt.Driver(k).i         = H(Nline, 2);
    Jnt.Driver(k).coortype  = H(Nline, 3);
    Jnt.Driver(k).j         = H(Nline, 4);
    if(Jnt.Driver(k).type ~= 3 && ...
            Jnt.Driver(k).type ~= 4 && ...
            Jnt.Driver(k).type ~= 5)
        disp('Type of driver not implemented');
    else 
        Jnt.Driver(k).spPi      = H(Nline, 5:6)';
        Jnt.Driver(k).spPj      = H(Nline, 7:8)';
        Jnt.Driver(k).order     = H(Nline, 9); % order of spline 
        Jnt.Driver(k).Filename  = H(Nline, 10);
    end
end
end

