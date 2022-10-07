clear;clc;
filename = "Material\Kinematics & Dynamics\trial_0013_G2.tsv";
outputfile = 'gait3d.gif';

header_info = readcell(filename, ...
    'FileType','text', ...
    'Delimiter','\t', ...
    'ExpectedNumVariables',2);

motion_data = table2array(readtable(filename, 'FileType','text', 'VariableNamingRule','preserve'));

no_frames   = cell2mat(header_info(1, 2));
no_cameras  = cell2mat(header_info(2, 2));
no_markers  = cell2mat(header_info(3, 2));
f           = cell2mat(header_info(4, 2));

% Pair of points that form bodies
connections = [1 2; 1 5; 2 5; 2 3; 3 4; 5 6; 6 7; 2 8; 8 14; 5 14; 8 9; 17 18; 16 18;
               9 10; 10 11; 11 12; 12 13; 14 15; 15 16; 16 17; 18 19; 10 12].';

% Steps of 4 frames to increase gif speed :)
for r = 1:4:length(motion_data)
    
    grid minor  % Adding grid lines
    plot_body(motion_data(r, :), connections)
    
    % Delay
    pause(0.01)
    % Saving the figure
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if r == 1
        imwrite(imind,cm,outputfile,'gif', 'Loopcount',inf,...
        'DelayTime',0.1);
    else
        imwrite(imind,cm,outputfile,'gif','WriteMode','append',...
        'DelayTime',0.1);
    end
end

