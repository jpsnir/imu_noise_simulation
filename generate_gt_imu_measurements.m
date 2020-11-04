function [accel_ideal,gyro_ideal] = generate_gt_imu_measurements(Fs,sim_Params)
%generate_gt_imu_measurements - simulates ideal (not having sensor noise) 
%acceleration and gyroscope values based on the simulation parameters and 
%sampling rate. 
%
% Syntax:  [accel_ideal,gyro_ideal] = generate_gt_imu_measurements(Fs,sim_Params)
%
% Inputs:
%    Fs - sampling rate
%    sim_params - struct that stores simulation parameters

% Outputs:
%    accel_ideal - column vector -
%                   simulated noise free acceleration values from the 
%                   waypoint trajectory

%    gyro_ideal - column vector -
%                  simulated noise free gyroscope values from the waypoint
%                  trajectory

% Example: 
%    Line 1 of example
%    Line 2 of example
%    Line 3 of example
%
% Other m-files required: None
% Subfunctions: None
% MAT-files required: none
%
% See also: waypointTrajectory,  OTHER_FUNCTION_NAME2
% Author: Jagatpreet Nir
% Work address: Northeastern Field Robotics Lab
% email: nir.j@northeastern.edu
% Website: http://www.
% Sept 2020; Last revision: 03-Nov-2020

waypoints = sim_Params.Trajectory.Waypoints;
toa = sim_Params.Trajectory.toa;
trajectory = waypointTrajectory(waypoints,toa,'SampleRate',Fs);


position = zeros(toa(end)*trajectory.SampleRate,3);
orientation = zeros(toa(end)*trajectory.SampleRate,1,'quaternion');
velocity = zeros(toa(end)*trajectory.SampleRate,3);
accel_ideal = zeros(toa(end)*trajectory.SampleRate,3);
gyro_ideal = zeros(toa(end)*trajectory.SampleRate,3);
count = 1;
while ~isDone(trajectory)
   [position(count,:),orientation(count,:),velocity(count,:),accel_ideal(count,:),gyro_ideal(count,:)] = trajectory();
   count = count + 1;
end


end

