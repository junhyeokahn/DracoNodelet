clear all
clc 
close all


%%
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;
jpos_des = fn_read_file(path, 'jpos_des', 1);
jvel_des = fn_read_file(path, 'jvel_des', 1);
mjpos = fn_read_file(path, 'mjpos', 1);
jjpos = fn_read_file(path, 'jjpos', 1);
mjvel = fn_read_file(path, 'mjvel', 1);
mjvel = fn_read_file(path, 'jjvel', 1);
time = fn_read_file(path, 'time', 1);

num_figure = 2;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end

x_min = 1.7;
x_max = 4.0;

figure(fig(1))
hold on
plot(time, jpos_des(1:length(time)) ,'r-','linewidth',3);
plot(time, jjpos(1:length(time)) ,'b-','linewidth',3);
plot(time, mjpos(1:length(time)) ,'c-','linewidth',3);
xlim([x_min, x_max])
%ylim([1.5, 2.5])
grid on
grid minor

figure(fig(2))
hold on
plot(time, jvel_des, 'r-', 'linewidth', 3);
plot(time, mjvel, 'b-', 'linewidth', 3);
grid on
grid minor