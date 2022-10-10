function plot_2d_data(LabData, Body, name)
    global NBody
    N = length(LabData.Coordinates(:,1));
    for n = 1:1
        plot(0, 0)
        
        grid on;
        hold on;
        for i = 1:2:(NBody*2 + 1)
            plot(LabData.Coordinates(n,i), LabData.Coordinates(n,i+1), 'ro')
        end
    
        for i = 1:NBody
            plot([LabData.Coordinates(n,2*(Body(i).pi)-1), LabData.Coordinates(n,2*(Body(i).pj)-1)], ...
                [LabData.Coordinates(n,2*(Body(i).pi)), LabData.Coordinates(n,2*(Body(i).pj))], 'b')
            plot(Body(i).x, Body(i).y, 'go')
        end


        ylim([0, 1.8])
        xlim([-2, 2])
        title('Running analysis', 'Interpreter', 'latex')
        xlabel('Position (m)', 'Interpreter', 'latex')
        ylabel('Position of markers (m)', 'Interpreter', 'latex')
        pause(0.01)
        hold off;
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if n == 1
            imwrite(imind,cm,['images\', name, '3d.gif'],'gif', 'Loopcount',inf,...
            'DelayTime',0.1);
        else
            imwrite(imind,cm,['images\', name, '3d.gif'],'gif','WriteMode','append',...
            'DelayTime',0.1);
        end
    end
end

