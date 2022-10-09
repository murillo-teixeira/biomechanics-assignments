function CutOffFrequency = get_cutoff_frequency(raw_data, FrequencyInterval, SamplingFrequency)

    N_f                 = length(FrequencyInterval);
    N                   = length(raw_data);
    Residuals           = zeros(length(FrequencyInterval), 1);
    R_Squared_Threshold = 0.97;

    for i = 1:length(FrequencyInterval)
        wn = (2 * FrequencyInterval(i)) / SamplingFrequency;
        [Ab, Bb] = butter(2, wn, 'low');
        filtered_data = filtfilt(Ab, Bb, raw_data);
        Residuals(i) = sqrt(sum((raw_data - filtered_data).^2)/N);
    end
    
    for n = N_f-2:-1:1
        mdl = fitlm(n:N_f, Residuals(n:N_f));
        r_squared = mdl.Rsquared.Ordinary;
        if r_squared <= R_Squared_Threshold
            break
        end
    end
    
    if r_squared > R_Squared_Threshold
        disp('Frequency not found')
    end
    
    coeffs = mdl.Coefficients.Estimate;
    [~, index] = min(abs(Residuals - coeffs(1)));
    CutOffFrequency = FrequencyInterval(index);
%     hold on;
%     plot(FrequencyInterval, Residuals);
%     regression_line = 100*coeffs(2)*FrequencyInterval + coeffs(1);
%     plot(FrequencyInterval, regression_line);
%     plot(FrequencyInterval, 1)
%     [~, index] = min(abs(Residuals - coeffs(1)));
%     
%     disp(FrequencyInterval(index))
%     plot(FrequencyInterval(index), Residuals(index), 'ro')
%     plot([FrequencyInterval(1), FrequencyInterval(N_f)],[Residuals(index), Residuals(index)], 'k--')
%     hold off;
end

