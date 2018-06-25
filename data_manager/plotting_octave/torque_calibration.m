clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

time = fn_read_file(path, 'curr_time', 1);
cmd_torque = fn_read_file(path, 'jeff_cmd', 2);
msrd_torque = fn_read_file(path, 'jeff', 2);
jpos = fn_read_file(path, 'motor_joint_pos', 2);

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

mass = 3.8;
r = 0.3;
theta = -0.17203;
g = 9.81;
gravity = mass * g * r * cos(jpos + theta);

figure(fig(1))
for i=1:2
subplot(2,1,i)
plot(time, cmd_torque(i,:) ,'r--','linewidth',3);
hold on
plot(time, msrd_torque(i,:), 'b-', 'linewidth',3);
hold on
%plot(time, gravity, 'g--', 'linewidth', 3);
hold off
end

set(gca, 'fontsize',12);
grid on

title('torque');
xlabel('Time (sec)','fontsize', 12);

figure(fig(2))
plot(time, jpos ,'b-','linewidth',3);

set(gca, 'fontsize',12);
grid on

title('Jpos');
xlabel('Time (sec)','fontsize', 12);