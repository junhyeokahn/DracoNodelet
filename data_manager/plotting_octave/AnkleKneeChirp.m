clc
clear all
close all

%% 
path = '/home/apptronik/ros/DracoNodelet/AnkleKneeController/ExperimentDataCheck';
fn_add_path('/home/apptronik/ros/DracoNodelet/data_manager/plotting_octave');


time = fn_read_file(path, 'time', 1);
input = fn_read_file(path, 'input', 1);
output = fn_read_file(path, 'output', 1);
cmd = fn_read_file(path, 'command', 1);

figure
plot(time, input)
figure
plot(cmd)