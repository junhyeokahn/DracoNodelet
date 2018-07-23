clc
clear all
close all

%% 
path = '/home/apptronik/ros/DracoNodelet/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/DracoNodelet/data_manager/plotting_octave');

time = fn_read_file(path, 'time', 1);
JPosAct = fn_read_file(path, 'JPosAct', 2);
JPosDes = fn_read_file(path, 'JPosDes', 2);

start_idx = 1;
end_idx = length(time) - 1000;
figure
hold on
plot(time(start_idx:end_idx), JPosDes(1, start_idx:end_idx), 'r')
plot(time(start_idx:end_idx), JPosAct(1, start_idx:end_idx), 'b')