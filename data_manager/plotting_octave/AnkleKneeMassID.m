clc
clear all
close all

%% 
path = '/home/apptronik/ros/DracoNodelet/AnkleKneeController/ExperimentDataCheck';
fn_add_path('/home/apptronik/ros/DracoNodelet/data_manager/plotting_octave');

theta_act = fn_read_file(path, 'massID_theta', 1);
theta_cmd = fn_read_file(path, 'massID_cmd_theta', 1);
trq = fn_read_file(path, 'massID_torque', 1);

figure
hold on
plot(theta_cmd, 'r')
plot(theta_act, 'b')

figure
hold on
plot(trq, 'r','linewidth',3)
trq_estimation1 = 3.8*0.35*9.81*cos(theta_act);
trq_estimation = 4.7*0.26425*9.81*cos(theta_act);
plot(trq_estimation, 'b', 'linewidth',3)
plot(trq_estimation1, 'g', 'linewidth',3)