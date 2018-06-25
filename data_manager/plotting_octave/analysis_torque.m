clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');


time = fn_read_file(path, 'curr_time', 1);
#joint_torque = fn_read_file(path, 'joint_effort', 2);
joint_torque = fn_read_file(path, 'jeff', 2);
cmd = fn_read_file(path, 'jeff_cmd', 2);
grav_test = fn_read_file(path, 'grav_debug',2);


original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 1;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end


%% Draw Figure
max_length = length(time) - 10;
time = time(1:max_length);
#q
figure(fig(1))

for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, joint_torque(i,1:max_length) ,'b-','linewidth',3);
  plot(time, grav_test(i,1:max_length), 'g--', 'linewidth', 3)
  plot(time, cmd(i,1:max_length), 'r--', 'linewidth',3);
  hold off

  set(gca, 'fontsize',12);
  grid on
  ylim([-40, 20])
end

title(tmp(1), 'torque');
xlabel('Time (sec)','fontsize', 12);
