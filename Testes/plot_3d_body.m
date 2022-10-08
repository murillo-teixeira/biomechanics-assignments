function plot_3d_body(row, connections)

    positions = [];
    plot3(1, 1, 1);
    hold on;
    view(21.800000000000004,7.200000000000001);
    grid minor  % Adding grid lines
    for n = 1:19
        X = row((n-1) * 3 + 3);
        Y = row((n-1) * 3 + 4);
        Z = row((n-1) * 3 + 5);
        positions(n, :) = [X, Y, Z];
        plot3(positions(n,1), positions(n,2), positions(n,3), 'ro')
    end
    for conn = connections
       plot3([positions(conn(1),1) positions(conn(2),1)], ...
             [positions(conn(1),2) positions(conn(2),2)], ...
             [positions(conn(1),3) positions(conn(2),3)], 'r')
    end
    ylim([-200 1000])
    xlim([-1500 2000])
    zlim([0 1800])
    
    hold off;
end

