function WriteModelInput(FileName, FileToBeRead)
global NBody Body Jnt Pts Frc SamplingFrequency
DataToWrite = zeros(1 + NBody + Jnt.NRevolute + Jnt.NDriver ...
    + Frc.NVarForceAppl + 3, 10);

% ... Initialize data
Nline = 1;

DataToWrite(Nline, 1)  = NBody;
DataToWrite(Nline, 2)  = Jnt.NRevolute;
DataToWrite(Nline, 3)  = Jnt.NTranslation;
DataToWrite(Nline, 4)  = Jnt.NRevRev;
DataToWrite(Nline, 5)  = Jnt.NTraRev;
DataToWrite(Nline, 6)  = Jnt.NCam;
DataToWrite(Nline, 7)  = Jnt.NGround;
DataToWrite(Nline, 8)  = Jnt.NSimple;
DataToWrite(Nline, 9)  = Jnt.NDriver;
DataToWrite(Nline, 10) = Pts.NPointsInt;
DataToWrite(Nline, 11) = Frc.NForceAppl;
DataToWrite(Nline, 12) = Frc.NSprDamper;
DataToWrite(Nline, 13) = Frc.NVarForceAppl;

%% Reading from MuboKap
MuboKAP = table2array(readtable(FileToBeRead, 'FileType','text', ...
    'VariableNamingRule','preserve'));

for i= 1:NBody
    Nline = Nline + 1;
    DataToWrite(Nline, 1) = Body(i).x;
    DataToWrite(Nline, 2) = Body(i).y;
    DataToWrite(Nline, 3) = Body(i).theta;
    DataToWrite(Nline, 4) = MuboKAP(1, 9*(i-1)+5);  % xd
    DataToWrite(Nline, 5) = MuboKAP(1, 9*(i-1)+6);  % yd
    DataToWrite(Nline, 6) = MuboKAP(1, 9*(i-1)+7);  % 0d
    DataToWrite(Nline, 7) = Body(i).mass;
    DataToWrite(Nline, 8) = Body(i).rg;
end

for k = 1 : Jnt.NRevolute 
    Nline = Nline + 1; 
    DataToWrite(Nline, 1) = Jnt.Revolute(k).i; 
    DataToWrite(Nline, 2) = Jnt.Revolute(k).j; 
    DataToWrite(Nline, 3:4) = Jnt.Revolute(k).spi * Body(Jnt.Revolute(k).i).Length; 
    DataToWrite(Nline, 5:6) = Jnt.Revolute(k).spj *Body(Jnt.Revolute(k).j).Length ;
end 

for k = 1 : Jnt.NDriver
    Nline                   = Nline + 1;         
    DataToWrite(Nline, 1)   = Jnt.Driver(k).type;
    DataToWrite(Nline, 2)   = Jnt.Driver(k).i;         
    DataToWrite(Nline, 3)   = Jnt.Driver(k).coortype;  
    DataToWrite(Nline, 4)   = Jnt.Driver(k).j;     
    DataToWrite(Nline, 5:6) = Jnt.Driver(k).spPi;
    DataToWrite(Nline, 7:8) = Jnt.Driver(k).spPj;
    DataToWrite(Nline, 9)   = Jnt.Driver(k).order;
    DataToWrite(Nline, 10)  = Jnt.Driver(k).Filename; 
end

%% ... Store information for force elements
% Stores the data regarding the force plates
for k = 1 : Frc.NVarForceAppl
    Nline = Nline + 1;
	% Bodies i and j
	i = Frc.VarForceAppl(k).i;
	
    DataToWrite(Nline, 1:3) = [i, Frc.VarForceAppl(k).order, ...
		Frc.VarForceAppl(k).Filename];
	
	% Writes the data to the files
	dlmwrite(sprintf('..\\MuboDAP\\VarForceAppl_%03d.txt', Frc.VarForceAppl(k).Filename), ...
		Frc.VarForceAppl(k).Data, 'delimiter', '\t', 'precision', 16, ...
		'newline', 'pc');
end

%% ... Store information about gravity
Nline = Nline + 1;
DataToWrite(Nline, 1:2) = [0, -9.81];

%% ... Store information about Baumgarte stabilization
Nline = Nline + 1; 
DataToWrite(Nline, 1:4) = [8, 1, 5, 5];

%% ... Store information about time
Nline = Nline + 1;
DataToWrite(Nline, 1) = MuboKAP(1, 1);
DataToWrite(Nline, 2) = 1/SamplingFrequency;
DataToWrite(Nline, 3) = MuboKAP(end, 1);
DataToWrite(Nline, 4) = 1/SamplingFrequency;

%% Finally, saving the matrix
writematrix(DataToWrite, FileName, 'Delimiter','tab');
end