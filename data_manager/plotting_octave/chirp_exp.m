clear all
clc 
close all

%% 
path = '/home/apptronik/ros/ForceCtrl_Test/force_ctrl_test/exp_data_check';
%path = '/home/apptronik/ros/apptronik_ros/experiment_server/log';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

cmd = fn_read_file(path, 'cmd', 1);
time_ns = fn_read_file(path, 'time', 1);
current = fn_read_file(path, 'current_A', 1);
loadcell = fn_read_file(path, 'loadcell_N', 1);
motor_pos = fn_read_file(path, 'motor_pos_rad', 1);
rubber_deflection = fn_read_file(path, 'rubber_deflection_um', 1);

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 2;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end

time = time_ns;
offset = 0;
for i =1:length(time_ns)-1
time(i) = time_ns(i) + offset;
if time_ns(i+1) < time_ns(i)
offset = offset + 1.e9;
end
end
time(end) = time_ns(end) + offset;

figure(fig(1))
plot(time, cmd ,'r-','linewidth',3);
%hold on
%plot(time, current, 'b--f_test_regression', 'linewidth',3);
%hold off

set(gca, 'fontsize',12);
grid on

title('torque');
xlabel('Time (sec)','fontsize', 12);