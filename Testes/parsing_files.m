clear;clc;
filename = "Material\Kinematics & Dynamics\trial_0001_static.tsv";

%% Reading raw data
header_info = readcell(filename, ...
    'FileType','text', ...
    'Delimiter','\t', ...
    'ExpectedNumVariables',2);

motion_data = table2array(readtable(filename, 'FileType','text', 'VariableNamingRule','preserve'));

%% Loading general info
no_frames   = cell2mat(header_info(1, 2));
no_cameras  = cell2mat(header_info(2, 2));
no_markers  = cell2mat(header_info(3, 2));
f           = cell2mat(header_info(4, 2));

%% Pair of points that form bodies
% connections = table2array(readtable("3d_connections.txt", "ExpectedNumVariables",2)).';

%% Creating 3d .gif
% create_3d_gif('run', motion_data, connections)

%% 2D plot of the body
% 
% grid on
% row = motion_data(1, :);
% hold on;
% for n = 1:19
%     X = row((n-1) * 3 + 3);
%     Z = row((n-1) * 3 + 5);
%     plot(X, Z, 'ro')
% end
% hold off;

%% Analysis of one coordinate for filtering

raw_data            = motion_data(:, 3);
time                = motion_data(:, 2);
FrequencyInterval   = (0.5:0.01:6.0).';

CutOffFrequency = get_cutoff_frequency(raw_data, FrequencyInterval, f);

wn = (2 * CutOffFrequency) / f;
[Ab, Bb] = butter(2, wn, 'low');
filtered_data = filtfilt(Ab, Bb, raw_data);
hold on;

subplot(2, 1, 1); 
plot(time, raw_data)
title('Dados brutos da posição da cabeça no eixo X')
ylabel('Position (mm)')
xlabel('Time(s)')

subplot(2, 1, 2); 
plot(time, filtered_data)
title('Dados filtrados da posição da cabeça no eixo X')
ylabel('Position (mm)')
xlabel('Time(s)')

hold off;