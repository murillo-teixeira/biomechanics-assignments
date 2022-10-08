function FilteredCoordinates = ReadProcessData(FileName)
global LabData
motion_data = table2array(readtable(FileName, 'FileType','text', 'VariableNamingRule','preserve'));
time                = motion_data(:, 2);
FrequencyInterval   = (0.5:0.01:6.0).';

header_info = readcell(FileName, ...
    'FileType','text', ...
    'Delimiter','\t', ...
    'ExpectedNumVariables',2)

%% Loading general info
no_frames   = cell2mat(header_info(1, 2));
no_cameras  = cell2mat(header_info(2, 2));
no_markers  = cell2mat(header_info(3, 2));
f           = cell2mat(header_info(4, 2));
i = 1;
j = 1;
while i<56
    CutOffFrequency = get_cutoff_frequency(motion_data(:, i+3), FrequencyInterval, f);
    wn = (2 * CutOffFrequency) / f;
    [Ab, Bb] = butter(2, wn, 'low');
    FilteredCoordinates(:,j) = filtfilt(Ab, Bb, raw_data);
    if rem(i,2) == 0
        i =i + 2;
    else
        i= i+1;
    end
    j=j+1;
end

