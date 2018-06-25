clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');


time = fn_read_file(path, 'curr_time', 1);
cmd_torque = fn_read_file(path, 'jeff_cmd', 2);
msrd_torque = fn_read_file(path, 'jeff', 2);
jpos = fn_read_file(path, 'motor_joint_pos',2);
jpos_des = fn_read_file(path, 'jpos_cmd',2);
jvel = fn_read_file(path, 'motor_joint_vel',2);
jvel_des = fn_read_file(path, 'jvel_cmd',2);
meff = fn_read_file(path, 'motor_current', 2);
aeff = fn_read_file(path, 'actuator_eff', 2);
aeff2 = fn_read_file(path, 'actuator_eff2', 2);
ee_des = fn_read_file(path, 'ee_des', 2);
ee_act = fn_read_file(path, 'ee_act', 2);
temp = fn_read_file(path, 'temperature', 2);

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 5;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end
%time = cpu_time/1000;
figure(fig(1))
for i=1:2
subplot(2,1,i)
plot(time, cmd_torque(i,:) ,'r--','linewidth',3);
hold on
plot(time, msrd_torque(i,:), 'b-', 'linewidth',3);
grid on

hold on
end
hold off
title('torque');
xlabel('Time (sec)','fontsize', 12);
grid on

figure(fig(2))
for i=1:2
subplot(2, 1, i)
hold on
plot(time, ee_des(i,:), 'r-', 'linewidth', 3);
plot(time, ee_act(i,:), 'b-', 'linewidth', 3);
if i==2
  ylim([0.5,1.0])
end
end
hold off
title('end effector')
grid on

figure(fig(3))
hold on
plot(ee_des(1, :), ee_des(2, :), 'r-', 'linewidth', 3);
plot(ee_act(1, :), ee_act(2, :), 'b-', 'linewidth', 3);
xlim([-0.1, 0.1])
ylim([0.5, 1])
axis equal
hold off
grid on

figure(fig(4))
for i=1:2
subplot(2, 1, i)
hold on
plot(time, meff(i,:), 'r-', 'linewidth', 3);
end
hold off
title('end effector')
grid on

figure(fig(5))
for i=1:2
subplot(2, 1, i)
hold on
plot(time, temp(i,:), 'r-', 'linewidth', 3);
end
hold off
title('end effector')
grid on