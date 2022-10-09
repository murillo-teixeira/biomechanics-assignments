clc;
FileName = '../Material/Kinematics & Dynamics/trial_0001_static.tsv';
motion_data = table2array(readtable(FileName, 'FileType','text', 'VariableNamingRule','preserve'));
raw_data = motion_data(:, 41);
FrequencyInterval   = (0.01:0.01:6.5).';
N_f                 = length(FrequencyInterval);
N                   = length(raw_data);
Residuals           = zeros(length(FrequencyInterval), 1);

SamplingFrequency = 200;

for i = 1:length(FrequencyInterval)
    wn = (2 * FrequencyInterval(i)) / SamplingFrequency;
    [Ab, Bb] = butter(2, wn, 'low');
    filtered_data = filtfilt(Ab, Bb, raw_data);
    Residuals(i) = sqrt(sum((raw_data - filtered_data).^2)/N);
end

plot(FrequencyInterval, Residuals);

for n = N_f-2:-1:1
    mdl = fitlm(n:N_f, Residuals(n:N_f));
    r_squared = mdl.Rsquared.Ordinary;
    if r_squared <= 0.97
        break
    end
end
disp(r_squared)

if r_squared > 0.97
    disp('Frequency not found')
end
% 
% coeffs = mdl.Coefficients.Estimate;
% [~, index] = min(abs(Residuals - coeffs(1)));
% CutOffFrequency = FrequencyInterval(index);
% disp(CutOffFrequency)
% hold on;
% plot(FrequencyInterval, Residuals);
% regression_line = (N/N_f)*coeffs(2)*FrequencyInterval + coeffs(1);
% plot(FrequencyInterval, regression_line);
% plot(FrequencyInterval, 1)
% [~, index] = min(abs(Residuals - coeffs(1)));
% 
% plot(FrequencyInterval(index), Residuals(index), 'ro')
% plot([FrequencyInterval(1), FrequencyInterval(N_f)],[Residuals(index), Residuals(index)], 'k--')
% hold off;