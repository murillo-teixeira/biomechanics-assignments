    function FilteredData = DoublePassLPFilter(ProcessedData, ...
    Frequency)

global SamplingFrequency

FrequencyInterval = (5.0:0.1:Frequency).';
CutOffFrequency = get_cutoff_frequency(ProcessedData, ...
    FrequencyInterval, SamplingFrequency);

wn = (2 * CutOffFrequency) / SamplingFrequency;
[Ab, Bb] = butter(2, wn, 'low');
FilteredData = filtfilt(Ab, Bb, ProcessedData);
end