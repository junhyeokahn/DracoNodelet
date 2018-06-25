clear all
clc 
close all

path = '/home/apptronik/Junhyeok';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

ankle = fn_read_file(path, 'ankle', 1);
knee = fn_read_file(path, 'knee', 1);

plot(ankle);
hold on
plot(knee)
hold off