function LabData = ReadProcessData(FileName)

global NBody;

motion_data = table2array(readtable(FileName, 'FileType','text', 'VariableNamingRule','preserve'));
time                = motion_data(:, 2);
FrequencyInterval   = (0.5:0.01:10.0).';

header_info = readcell(FileName, ...
    'FileType','text', ...
    'Delimiter','\t', ...
    'ExpectedNumVariables',2);

%% Loading general info
no_frames   = cell2mat(header_info(1, 2));
no_cameras  = cell2mat(header_info(2, 2));
no_markers  = cell2mat(header_info(3, 2));
f           = cell2mat(header_info(4, 2));
FilteredCoordinates = zeros(length(motion_data(:, 1)), NBody * 3);
CutOffFrequencies   = zeros(NBody * 3);
i = 1;
j = 1;

while i < NBody * 3
    CutOffFrequencies(j) = get_cutoff_frequency(motion_data(:, i + 3), FrequencyInterval, f);
    wn = (2 * CutOffFrequencies(j)) / f;
    [Ab, Bb] = butter(2, wn, 'low');
    FilteredCoordinates(:, j) = filtfilt(Ab, Bb, motion_data(:, i + 3));
    if rem(i, 2) == 0
        i = i + 2;
    else
        i = i + 1;
    end
    j = j + 1;
end


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