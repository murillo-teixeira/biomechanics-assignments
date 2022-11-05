function WriteModelInput(FileName)
global NBody Body Jnt Pts Frc
DataToWrite = zeros(1 + NBody + Jnt.NRevolute + Jnt.NDriver, 10);

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

for i= 1:NBody
    Nline = Nline + 1;
    DataToWrite(Nline, 1) = Body(i).x;
    DataToWrite(Nline, 2) = Body(i).y;
    DataToWrite(Nline, 3) = Body(i).theta;
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

	% Bodies i and j
	i = Frc.VarForceAppl(k).i;
	
	fprintf(fid, '%d %d %d\r\n', i, Frc.VarForceAppl(k).order, ...
		Frc.VarForceAppl(k).Filename);
	
	% Writes the data to the files
	dlmwrite(sprintf('VarForceAppl_%03d.txt', Frc.VarForceAppl(k).Filename), ...
		Frc.VarForceAppl(k).Data, 'delimiter', '\t', 'precision', 16, ...
		'newline', 'pc');
end

writematrix(DataToWrite, FileName, 'Delimiter','tab');
end