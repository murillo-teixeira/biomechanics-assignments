function plot_2d_data(LabData, Body)
    global NBody

    hold on;
    for i = 1:2:NBody*2
        plot(LabData.Coordinates(1,i), LabData.Coordinates(1,i+1), 'ro')
    end
    hold off;
end

