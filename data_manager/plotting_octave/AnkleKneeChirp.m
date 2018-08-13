clc
clear all
close all

%% 
path = '/home/apptronik/ros/DracoNodelet/AnkleKneeController/ExperimentDataCheck';
fn_add_path('/home/apptronik/ros/DracoNodelet/data_manager/plotting_octave');


time = fn_read_file(path, 'time', 1);
input = fn_read_file(path, 'input', 1);
output = fn_read_file(path, 'output', 1);
debug_q = fn_read_file(path, 'debug_knee_q', 1);
debug_qdot = fn_read_file(path, 'debug_knee_qdot', 1);
debug_effort = fn_read_file(path, 'debug_knee_effort', 1);
debug_bus_voltage = fn_read_file(path, 'debug_bus_voltage', 1);
debug_current_msr = fn_read_file(path, 'debug_current_msr', 1);
temperature = fn_read_file(path, 'temperature', 1);

figure
plot(input)
figure
plot(output)
figure
plot(temperature)
figure
plot(debug_q)
figure
plot(debug_qdot)
figure
plot(debug_effort)
figure
plot(debug_current_msr)
figure
plot(debug_bus_voltage)