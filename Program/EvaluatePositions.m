function Position = EvaluatePositions(LabData)
    global NBody Body Jnt
    frame = 1;
    Position = zeros(length(LabData.Coordinates(:, 1)), NBody * 3);

    for n=1:length(LabData.Coordinates(:,1))
        j = 1;
        for i=1:3:NBody*3
            csi_temp = [LabData.Coordinates(n,2*(Body(j).pj)-1) - ... %x_i
                        LabData.Coordinates(n,2*(Body(j).pi)-1), ...  %x_j
                        LabData.Coordinates(n,2*(Body(j).pj)) - ...  %y_i
                        LabData.Coordinates(n,2*(Body(j).pi))];       %y_j

            csi_temp = csi_temp/norm(csi_temp);
            % eta
            eta_temp = [-csi_temp(2),csi_temp(1)];
            
            % theta
            theta = atan2(csi_temp(2), csi_temp(1));
            if theta < 0
                theta = 2*pi + theta;
            end
            A =[cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            Position(n, i:i+1) = [LabData.Coordinates(n,2*(Body(j).pi)-1); LabData.Coordinates(n,2*(Body(j).pi))] + ...
                A*[Body(j).Length*Body(j).PCoM; 0];

            if n == frame
                Body(j).x = Position(n, i);
                Body(j).y = Position(n, i+1);
                Body(j).theta = theta;
            end
            Position(n, 3*j) = theta;
            j=j+1;

%             Body(i).csi_x = csi_temp(1);
%             Body(i).csi_y = csi_temp(2);
%             Body(i).eta_x = eta_temp(1);
%             Body(i).eta_y = eta_temp(2);
        end
    end
end

