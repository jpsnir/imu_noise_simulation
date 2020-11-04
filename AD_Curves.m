function [tau, adev, avar, adev_upperbound,adev_lowerbound, numSamples] = AD_Curves(data,Fs)
    actual_data = data;
    numSamples = length(actual_data);

    % Determine Fs from data timestamps
    % Fs = N/(actual_data.data_imu(N+1,1) - actual_data.data_imu(1,1))

    % time_diff = diff(actual_data.data_imu(:,1));
    % Fs = N/sum(time_diff(1:N)); choose N = 100, 1000
    t = 0:1/Fs:(numSamples-1)/Fs;

    % Actual data model
    [avar,tau] = allanvar(actual_data,'octave',Fs);
    adev = avar.^0.5;

    %% Quality of estimation
    clusters = numSamples./(Fs*tau);
    percentage_error = 1./sqrt(2*(clusters-1));
    adev_upperbound = (1+percentage_error).*adev;
    adev_lowerbound = (1-percentage_error).*adev;
end