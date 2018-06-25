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
core_temp = fn_read_file(path, 'core_temperature', 1);
springDef = fn_read_file(path, 'measured_torque', 1);
loadcell = fn_read_file(path, 'load_cell', 1);
meff_measure = fn_read_file(path, 'motor_effort', 1);
meff_des = fn_read_file(path, 'torque_command', 1);
time = fn_read_file(path, 'time', 1);

num_figure = 3;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end

figure(fig(1))
hold on
plot(time, springDef,'r-','linewidth',3);
plot(time, loadcell,'b-','linewidth',3);
grid on
grid minor

figure(fig(2))
hold on
plot(time, core_temp, 'r-', 'linewidth', 3);
grid on
grid minor

figure(fig(3))
hold on
plot(time, meff_des, 'r-', 'linewidth', 3);
plot(time, meff_measure, 'b-', 'linewidth', 3);
grid on
grid minor