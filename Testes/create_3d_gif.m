function create_3d_gif(name, motion_data, connections)

    for r = 1:4:length(motion_data)
        grid minor  % Adding grid lines
        plot_3d_body(motion_data(r, :), connections)
        
        % Delay
        pause(0.01)
        % Saving the figure
        frame = getframe(gcf);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if r == 1
            imwrite(imind,cm,['images\', name, '3d.gif'],'gif', 'Loopcount',inf,...
            'DelayTime',0.1);
        else
            imwrite(imind,cm,['images\', name, '3d.gif'],'gif','WriteMode','append',...
            'DelayTime',0.1);
        end
    end

end

