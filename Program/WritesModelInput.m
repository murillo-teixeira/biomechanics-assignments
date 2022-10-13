function WriteModelInput(FileName)
global NBody Body Jnt Pts
DataToWrite = zeros(1 + NBody + Jnt.NRevolute, 10);

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
writematrix(DataToWrite, FileName, 'Delimiter','tab');
end