% This script generates the theoretical AD curves from the model parameters
% obtained from yaml files and the simulation case selected. To run this
% script,
% Run 
% >> theoretical_AD_curves
% Other m-files required: compute_theoretical_AD_curve.m,
%                         YAML folder to read yaml files. 
% Subfunctions: None
% MAT-files required: none
% Yaml-files required: 1) In folder IMU_params/test_imu_models/
%                      2) In folder Sim_params/test_motion - stationary.yaml

% Author: Jagatpreet Nir
% Work address: Northeastern Field Robotics Lab
% email: nir.j@northeastern.edu
% Website: http://www.
% Sept 2020; Last revision: 03-Nov-2020


clear all;
close all;

% all the script paths that are required.
script_paths;

% The folder stores .yaml files that stores different models
% of IMU. 
imu_parameter_folder = 'IMU_params/test_imu_models/';

% The file contains the parameters for stationary IMU simulation. It also
% specifies which model to choose 'm' - chooses MATLAB IMU object model
% 'p' - chooses model coded 
sim_parameter_filepath = 'Sim_params/test_motion/stationary.yaml';

% Choose the cases for which you need to compute the allan deviation plot.
% w - white noise, b - brown noise, p - pink noise
% acc - accelerometer, gyro - gyroscope
% refer the folder: IMU_params/test_imu_models 
% to see different noise models
% and their parameters. 
simulation_case = struct('w_acc',1,... %1
                         'b_acc',1,... %2
                         'p_acc',1,... %3
                         'p_acc_1',0,...
                         'w_gyro',0,...%4
                         'b_gyro',0,...%5
                         'p_gyro',0,...%6
                         'wb_acc',1,...%7
                         'wb_acc_gyro',0,...%8
                         'wbp_acc',1,...%9
                         'wbp_acc_1',0,...
                         'wbp_acc_gyro',0); %10
                     
f = fieldnames(simulation_case);
a = [];

% get all the field names with input 1
for k = 1:length(f)
    f_ = getfield(simulation_case,f{k});
    if  f_ == 1
        a = [a,f(k)]; % append and increase the size of a;
    end
end
f = a; % store in f

% Computes the theoretical Allan deviation curve for the models selected in
% simulation_case variable where the fields are 1. 
for i = 1:length(f)
    imu_parameter_file = strcat('sim_test_imu_',f(i),'.yaml');
    imu_parameter_filepath{i} = fullfile(imu_parameter_folder,imu_parameter_file{1});
    imu_Params{i} = YAML.read(imu_parameter_filepath{i});
    [ad{i},t{i}] = compute_theoretical_AD_curve(imu_Params{i},'a');
end

% find white and brown noise parameter from the AD Plot. 
tau_1 = t{1}((find(t{2} == 1)));
tau_3 = t{2}((find(t{2} == 3)));
K = ad{2}((find(t{2} == 3)));
N = ad{1}((find(t{1} == 1)));

% Plot AD on a log log scale. white and brown noise are plotted 
style = {'--b','--r','--m','-c','-g'};
legend_strings = {'White','Brown','Pink','white-brown','white-brown-pink'};
for i = 1:length(f)
    loglog(t{i},ad{i},style{i},'Linewidth',2);
    hold on;
end

% Plot the points which depict the parameter of 
% noise density and randomwalk on the AD curve. 
loglog(tau_1,N,'.k',tau_3,K,'.k','MarkerSize',20);
legend(legend_strings(1:length(f)));
xlabel('\tau')
ylabel('\sigma(\tau)')
title('Allan Deviation - theoretical')
grid on
