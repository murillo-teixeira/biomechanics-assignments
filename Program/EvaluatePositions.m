function Position = EvaluatePositions(LabData)
    global NBody Body
    Position = zeros(length(LabData.Coordinates(:, 1)), NBody * 5);
    
    for n=1:length(LabData.Coordinates(:,1))
        for i=1:NBody
            % csi
            csi_temp = [LabData.Coordinates(n,2*(Body(i).pi)-1) - ... %x_i
                   LabData.Coordinates(n,2*(Body(i).pj)-1), ...  %x_j
                   LabData.Coordinates(n,2*(Body(i).pi)) - ...  %y_i
                   LabData.Coordinates(n,2*(Body(i).pj))];       %y_j
            
            Position(n, i:i+1) = csi_temp/norm(csi_temp);

            % eta
            Position(n, i+2:i+3) = [-Position(n,i+1), Position(n, i)];
    
            % theta
            Position(n, i+4) = 2*pi - atan2(norm(cross([Position(n,i), Position(n,i+1), 0], [1,0, 0])), dot(Position(n,i:i+1), [1,0]));
        end
    end
end

