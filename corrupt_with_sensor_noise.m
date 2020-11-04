function [accel,gyro,nw,b] = corrupt_with_sensor_noise(ideal_acc,ideal_gyro,...
                                                        w_R_i_ideal,...
                                                        imu_params,...
                                                        sim_params)
%corrupt_with_sensor_noise - corrupts the ideal measurements with different 
%noise sources
%Optional file header info (to give more details about the function than in the H1 line)
%Optional file header info (to give more details about the function than in the H1 line)
%Optional file header info (to give more details about the function than in the H1 line)
%
% Syntax:  [accel,gyro,nw,b] = corrupt_with_sensor_noise(ideal_acc,ideal_gyro,...
%                                                    w_R_i_ideal,...
%                                                    imu_params,...
%                                                   sim_params)
%
% Inputs:
%    ideal_acc - noise free acceleration values obtained from 
%    ideal_gyro - noise gree angular velocities values in sensor frame
%    w_R_i_ideal - noise free rotation matrix of the current position.:
%                  Current code only supports a stationary simulation of IMU
%    imu_params - struct that stores IMU parameters
%    sim_params - struct that stores simulation parameters

% Outputs:
%    accel - noise corrupted sensor acceleration measurements - column
%    vector
%    gyro - noise corrupted sensor gyroscope measurements - column vector

%    nw   - white noise signal that corrupts the ideal measurements
%           struct : nw.acc and nw.gyro. Each entry stores a 
%           [3x1] column vector for x,y and z directions. 
%    b    - bias signal due to brown+pink noise in ideal measurements.
%           struct : b.acc and b.gyro. Each entry stores a 
%           [3x1] column vector for x,y and z directions.
% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Jagatpreet Nir
% Work address: Northeastern Field Robotics Lab
% email: nir.j@northeastern.edu
% Website: http://www.
% Sept 2020; Last revision: 03-Nov-2020 
global g_world;
[M,N] = size(ideal_acc);
del_t = 1/sim_params.SamplingRate;


% White noise : Strength of noise due to sampling is
% noise_density/sqrt(sampling_time)
nw.acc = imu_params.Accelerometer.NoiseDensity.*randn(M,3)/sqrt(del_t); % Mx3
nw.gyro = imu_params.Gyroscope.NoiseDensity.*randn(M,3)/sqrt(del_t); % Mx3

% Random Walk:  Strength of noise due to sampling is
% noise_density/sqrt(sampling_time)

nb.acc = imu_params.Accelerometer.RandomWalk.*randn(M,3)/sqrt(del_t); % Mx3
nb.gyro = imu_params.Gyroscope.RandomWalk.*randn(M,3)/sqrt(del_t); % Mx3

% Bias instability: Strength of noise due to sampling is
% noise_density/sqrt(sampling_time)
np.acc = imu_params.Accelerometer.BiasInstability.*randn(M,3)/sqrt(del_t); % Mx3
np.gyro = imu_params.Gyroscope.BiasInstability.*randn(M,3)/sqrt(del_t); % Mx3

% time constant
tau.acc = imu_params.Accelerometer.tau; % 1x3
tau.gyro = imu_params.Gyroscope.tau; % 1x3

% delta decay
% If the value of tau is zero, then we assume there is no pink/flicker
% noise in the model.
if tau.acc == 0
    phi.acc = 0;
else
    phi.acc = 1./tau.acc; % 1x3
end

if tau.gyro == 0
    phi.gyro = 0;
else
    phi.gyro = 1/tau.gyro; % 1x3
end


% Initial bias
b_0.acc = imu_params.Accelerometer.b_on;
b_0.gyro = imu_params.Gyroscope.b_on;


% Preallocate
b.acc = zeros(M,3);
b.gyro = zeros(M,3);

% Bias model with brown and pink noise
for i = 1:M
    if i == 1
        b.acc(i,:) = b_0.acc;
        b.gyro(i,:) = b_0.gyro;
    else
%             b.acc(i,:) = b.acc(i-1,:) - phi.acc.*b.acc(i-1,:) + nb.acc(i,:) + np.acc(i,:);
%             b.gyro(i,:) = b.gyro(i-1,:) - phi.gyro.*b.gyro(i-1,:) + nb.gyro(i,:) + np.gyro(i,:);
          b.acc(i,:) = b.acc(i-1,:) - del_t*phi.acc.*b.acc(i-1,:) + del_t*(nb.acc(i,:) + np.acc(i,:));
          b.gyro(i,:) = b.gyro(i-1,:) - del_t*phi.gyro.*b.gyro(i-1,:) + del_t*(nb.gyro(i,:) + np.gyro(i,:));
    end
end

%% IMU Model
gyro = ideal_gyro + b.gyro + nw.gyro;
accel = ideal_acc + b.acc + nw.acc + (w_R_i_ideal*g_world)';
end
