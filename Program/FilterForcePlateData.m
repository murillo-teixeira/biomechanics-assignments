function ProcessedData = FilterForcePlateData(ProcessedData, SamplingFrequency)

% Considering only the vertical component of the force, I will
% estimate the instants of time in which contact existe by finding
% the time steps for which the force was larger than 5 N

ContactTimeSteps = fin(ProcessedData(:,2) > 5);

% Filters the forces
for j = 1 : 2
	
	% Filters the force data using a low pass filter with a cut-off
	% frequency of 20 Hz
	FilteredData = DoublePassLPFilter(ProcessedData(:,2), ...
		20, SamplingFrequency);

	% the instants of time for which no contact existed will be 
	% assigned a 0 N force
	FilteredData(~ContactIndices) = 0;

	% Update the output
	ProcessedData(:, j) = FilteredData;

end

% Filters the center of pressure
for j = 3 : 4
	
	% Before filtering, the position of the center of pressure
	% immediately before and after contact will be put at those
	% positions to diminish the impact of the filter adaptation
	RawCoP = ProcessedData(:, j);
	if(ContactTimeSteps(1) > 1)
		% Puts the position for all time steps before contact equal to
		% the position of the first contact
		RawCoP(1 : ContactTimeSteps(1) - 1) = RawCop(ContactTimeSteps(1));
	end
	if (ContactTimeSteps(end) < length(RawCoP))
		% Puts the position for all time steps after contact equal to
		% the position of the last contact
		RawCoP(ContactTimeSteps(end) + 1 : end) = RawCoP(ContactTimeSteps(end));
	end

	% Filters the center of pressure using a low pass filter with a 
	% cut-off frequency of 10 Hz
	FilteredData = DoublePassLPFilter(RawCoP, 10, SamplingFrequency);
    
	% Update the output
	ProcessedData(:, j) = FilteredData;
	
	% Plots of the results
	% 	plot(RawCoP); hold on; plot(FilteredData, 'r'); hold off;
	% pause
end