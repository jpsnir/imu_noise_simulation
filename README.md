# imu_noise_simulation
Simulates an IMU noise model for a stationary IMU and generates AD curves for comparison.
This repository is tested to work with **MATLAB 2019 b or greater** 

To get the theoretical AD curves, run the following on your matlab command line 
> theoretical_AD_curves

To get the Allan deviation curve from the matlab and IMU models, run the following scripts on matlab command line
> sensor_models

This takes a while to run and stores the results in simulation_results folder as a .mat file. 

Then run on matlab command line
>Compare_models

to generate plots showing comparison between white and brown noise.

The function script *corrupt_with_sensor_noise.m* is the IMU model that we coded up. 

The function script *simulate_motion.m* generates acceleration and gyroscope samples either from the matlab IMU object or our model 
in *corrupt_with_sensor_noise.m*

The config files in IMU_params/test_imu_params stores parameters for different IMU models with noise model parameters. 

