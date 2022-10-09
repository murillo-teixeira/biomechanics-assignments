function Position = EvaluatePositions(GaitData)
global NBody
Position = zeros(length(GaitData.Coordinates(:, 1)), NBody * 5);

for n=1:length(GaitData.Coordinates(:,1))
    for i=1:NBody
        % csi
        Position(n, i:i+1) = [LabData.Coordinates(n,2*(Body(i).pi)-1) - LabData.Coordinates(n,2*(Body(i).pj)-1), ... 
            LabData.Coordinates(n,2*(Body(i).pi)) - LabData.Coordinates(n,2*(Body(i).pj))]...
            /(sqrt((LabData.Coordinates(n,2*(Body(i).pi)-1) - LabData.Coordinates(n,2*(Body(i).pj)-1)^2 + ...
            LabData.Coordinates(n,2*(Body(i).pi)) - LabData.Coordinates(n,2*(Body(i).pj))^2)));
        
        % eta
        Position(n, i+2:i+3) = [-Position(n,i+1), Position(n, i)];

        % theta
        Position(n, i+4) = 360 - atan2(norm(cross(Position(n,i:i+1), [1,0], 2)), dot(Position(n,i:i+1), [1,0]));
    end
end
%head_csi = (GaitData(2) - GaitData(1))/(sqrt((GaitData(2) - GaitData(1))'*(GaitData(2)-GaitData(1))));
%head_eta = [-head_csi(1,2), head_csi(1,1)]
%head_theta = 360 - atan2(norm(cross(head_csi, head_eta)), dot(head_csi, head_eta))
end

