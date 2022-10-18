FileName = "./BiomechanicalModel.out";
motion_data = table2array(readtable(FileName, 'FileType','text', 'VariableNamingRule','preserve'));
for i = 4:9:232
    figure(floor(i/9)+1)
    plot(motion_data(:, 1), motion_data(:, i))
end