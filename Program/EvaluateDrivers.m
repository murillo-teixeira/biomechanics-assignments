function EvaluateDrivers(Positions)
     global Jnt Times

     for k=1:Jnt.NDriver
           if(Jnt.Driver(k).type == 3)
               writematrix([Times, Positions(:, (Jnt.Driver(k).i-1)*3 + Jnt.Driver(k).coortype)], ...
                   "Driver_"+ num2str(Jnt.Driver(k).Filename, '%03d')+ ".txt", 'Delimiter','tab')
           else
               writematrix([Times, Positions(:, (Jnt.Driver(k).j-1)*3 + 3) - Positions(:, (Jnt.Driver(k).i-1)*3 + 3)], ...
                   "Driver_" + num2str(Jnt.Driver(k).Filename, '%03d') + ".txt", 'Delimiter','tab')
           end
     end
end

