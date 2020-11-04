function [t,accelSamples,gyroSamples,varargout] = simulate_motion(imu_params,sim_params)
%simulate_motion - simulates IMU measurements - linear acceleration and 
%angular velocities
%
% Syntax:  [t,accelSamples,gyroSamples,varargout] = simulate_motion(imu_params,sim_params)
%
% Inputs:
%    imu_params - struct that stores IMU parameters
%    sim_params - struct that stores simulation parameters

% Outputs:
%    t - column vector that stores the time
%    accelSamples - column vector -
%                   simulated acceleration values obtained from IMU model 
%    gyroSamples - column vector -
%                  simulated gyroscope values obtained from IMU model
%    varargout - cell array : assigned in line 77,78 in the else block. 
%                The values are output for debugging purposes for testing
%                corrupt_with_sensor_noise.m function. 

% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: corrupt_with_sensor_noise.m,
%                         generate_gt_imu_measurements.m

% Subfunctions: createParamsObject(imu_Params,type)
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Jagatpreet Nir
% Work address: Northeastern Field Robotics Lab
% email: nir.j@northeastern.edu
% Website: http://www.
% Sept 2020; Last revision: 03-Nov-2020

Fs = sim_params.SamplingRate; % sampling rate
simTime = sim_params.TotalTime;
t = (0:1/Fs:simTime)'; % a column vectors

% Do a stationary simulation
if sim_params.isStationary == 1  

    accel_ideal_B = zeros(length(t),3); % Generated in sensor frame
    gyro_ideal_B = zeros(length(t),3); % Generated in sensor frame

    % Orientation with respect to world frame
    w_R_i_ideal = eul2rotm(deg2rad(sim_params.InitialCondition.eul_w_0)); 
else % Do a waypoint trajectory
    % Generated in sensor frame
    % Not required for understanding noise models.
    [accel_ideal_B,gyro_ideal_B] = generate_gt_imu_measurements(Fs,sim_params);
end

acc_params = createParamsObject(imu_params,'a');
gyro_params = createParamsObject(imu_params,'g');
str = strcat('Displaying gyro and acc params that are sent \n',...
              'to imu object and corrupt_with_sensor_noise function');
disp(str);
disp(acc_params);
disp(gyro_params);
%disp('Press a key to continue');
pause(.5);

% load imu sensor object
% Generated in sensor frame
imu = imuSensor('accel-gyro',...
    'SampleRate',Fs,...
    'Accelerometer',acc_params,...
    'Gyroscope',gyro_params);

if sim_params.model_type == 'm'
    % Variable output assignment is not done in this block.
    [accelSamples,gyroSamples] = imu(accel_ideal_B,gyro_ideal_B);
else
    % Noisy measurements generated in sensor frame
    % Variable argument output assignment are done only in this function. 
    [accelSamples,gyroSamples,...
    varargout{1},varargout{2}] = corrupt_with_sensor_noise(accel_ideal_B,...
                                                  gyro_ideal_B,...
                                                  w_R_i_ideal,...
                                                  imu_params,...
                                                  sim_params);
end

end

function params = createParamsObject(imu_Params,type)
    if type == 'a'
     params = accelparams('MeasurementRange',imu_Params.Accelerometer.MeasurementRange, ...
    'Resolution',imu_Params.Accelerometer.Resolution, ...
    'ConstantBias',imu_Params.Accelerometer.ConstantBias, ...
    'AxesMisalignment',imu_Params.Accelerometer.AxesMisalignment, ...
    'NoiseDensity',imu_Params.Accelerometer.NoiseDensity, ...
    'BiasInstability',imu_Params.Accelerometer.BiasInstability, ...
    'RandomWalk',imu_Params.Accelerometer.RandomWalk,...
    'TemperatureBias', imu_Params.Accelerometer.TemperatureBias, ...
    'TemperatureScaleFactor', imu_Params.Accelerometer.TemperatureScaleFactor);
    elseif type == 'g'
     params = gyroparams('MeasurementRange',imu_Params.Gyroscope.MeasurementRange, ...
    'Resolution',imu_Params.Gyroscope.Resolution, ...
    'ConstantBias',imu_Params.Gyroscope.ConstantBias, ...
    'AxesMisalignment',imu_Params.Gyroscope.AxesMisalignment, ...
    'NoiseDensity',imu_Params.Gyroscope.NoiseDensity, ...
    'BiasInstability',imu_Params.Gyroscope.BiasInstability, ...
    'RandomWalk',imu_Params.Gyroscope.RandomWalk,...
    'TemperatureBias', imu_Params.Gyroscope.TemperatureBias, ...
    'TemperatureScaleFactor', imu_Params.Gyroscope.TemperatureScaleFactor);    
    end
end
