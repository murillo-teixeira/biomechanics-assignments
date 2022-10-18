function LabData = ReadProcessData(FileName)

global Times NBody;

motion_data = table2array(readtable(FileName, 'FileType','text', 'VariableNamingRule','preserve'));
Times               = motion_data(:, 2);
% Comentar com o professor o gráfico de frequências de corte
FrequencyInterval   = (1.0:0.1:6.5).';

header_info = readcell(FileName, ...
    'FileType','text', ...
    'Delimiter','\t', ...
    'ExpectedNumVariables',2);

%% Loading general info
no_frames   = cell2mat(header_info(1, 2));
no_cameras  = cell2mat(header_info(2, 2));
no_markers  = cell2mat(header_info(3, 2));
f           = cell2mat(header_info(4, 2));
FilteredCoordinates = zeros(length(motion_data(:, 1)), 19 * 2);
CutOffFrequencies   = zeros(0, 19 * 2);

j = 1;
for i = 3:59
    if rem(i, 3) == 1
        continue
    end
    CutOffFrequencies(j) = get_cutoff_frequency(motion_data(:, i), FrequencyInterval, f);
    
    wn = (2 * CutOffFrequencies(j)) / f;
    [Ab, Bb] = butter(2, wn, 'low');
    FilteredCoordinates(:, j) = filtfilt(Ab, Bb, motion_data(:, i));
    j = j + 1;
end

label = {'Head', 'Left Shoulder', 'Left Elbow', 'Left Wrist', 'Right Shoulder', ...
        'Right Elbow', 'Right Wrist', 'Left Hip', 'Left Knee', 'Left Ankle', ...
        'Left Heel', 'Left Meta V', 'Left Toe II', 'Right Hip', 'Right Knee', ...
        'Right Ankle', 'Right Heel', 'Right Meta V', 'Right Toe II'};
hold off
figure(2)
CutOffFrequencies   = reshape(CutOffFrequencies, [], 2);
b = bar(reordercats(categorical(label), label), CutOffFrequencies);
b(2).FaceColor = '#0072BD';
b(1).FaceColor = '#FF0000';
xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
'VerticalAlignment','bottom')

xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(b(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

title('Choice of Cutoff Frequency', 'Interpreter', 'latex')
xlabel('Coordinates Points', 'Interpreter', 'latex')
ylabel('Cutoff Frequency [Hz]', 'Interpreter', 'latex')
legend({'X coord','Z coord'},'Location','southeast')

% Organizes the data according to the definition of the biomechanical model % Notice that the coordinates from the lab are organized as follows: % 1 - Head; % 2 - L_Shoulder; % 3 - L_Elbow; % 4 - L_Wrist; % 5 - R_Shoulder % 6 - R_Elbow; % 7 - R_Wrist; % 8 - L_Hip; % 9 - L_Knee; % 10 - L_Ankle; % 11 - L_Heel; % 12 - L_Meta_V; is 13 - L_Toe_II; % 14 - R_Hip; % 15 - R_Knee % 16 - R_Ankle; % 17 - RHeel; % 18 - R_Meta_V; % 19 - R_Toe_II
LabData.Coordinates = [FilteredCoordinates(:,1:2), ... % Head 
                       (FilteredCoordinates(:,3:4) + FilteredCoordinates(:,9:10)) / 2,... % Midpoint between shoulders 
                       FilteredCoordinates(:,11:12),... % Right elbow 
                       FilteredCoordinates(:,13:14), ... % Right wrist
                       FilteredCoordinates(:,5:6),...   % Left elbow
                       FilteredCoordinates(:,7:8),...   % Left wrist
                       (FilteredCoordinates(:, 15:16) + FilteredCoordinates(:,27:28)) / 2, ... % Midpoint between hips
                       FilteredCoordinates(:,29:30), ... % Right knee
                       FilteredCoordinates(:,31:32), ... % Right ankle
                       FilteredCoordinates(:,35:36), ... % Right metatarsal
                       FilteredCoordinates(:,37:38), ... % Right hallux
                       FilteredCoordinates(:,17:18), ... % Left knee
                       FilteredCoordinates(:,19:20), ... % Right ankle
                       FilteredCoordinates(:,23:24), ... % Right metatarsal
                       FilteredCoordinates(:,25:26)] * 1e-3; % Right hallux

end