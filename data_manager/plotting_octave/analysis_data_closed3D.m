clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');


time = fn_read_file(path, 'curr_time', 1);
config = fn_read_file(path, 'config', 3);
qdot = fn_read_file(path, 'qdot', 3);
config_des = fn_read_file(path, 'jpos_cmd', 2);
qdot_des = fn_read_file(path, 'jvel_cmd', 2);
eff_des = fn_read_file(path, 'jeff_cmd', 2);
eff_act = fn_read_file(path, 'joint_eff', 2);
ee_act = fn_read_file(path, 'ee_act_debug', 2);
ee_des = fn_read_file(path, 'ee_des_debug', 2);
grav = fn_read_file(path, 'gravity', 2);
ctrl_time = fn_read_file(path, 'ctrl_time_debug', 1);
err_sum = fn_read_file(path, 'err_sum',2);
core_temp = fn_read_file(path, 'core_temp', 2);
ankle_efficiency = fn_read_file(path, 'ankle_efficiency', 3);
knee_efficiency = fn_read_file(path, 'knee_efficiency', 3);

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 8;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end


%% Draw Figure

#q
figure(fig(1))

for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, config(i,:) ,'b-','linewidth',3);
  plot(time, config_des(i,:), 'r--', 'linewidth',3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Q');
xlabel('Time (sec)','fontsize', 12);

# qdot
figure(fig(2))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, qdot(i,:) ,'b-','linewidth',3);
  hold on
  plot(time, qdot_des(i,:), 'r--', 'linewidth',3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Qdot');
xlabel('Time (sec)','fontsize', 12);

# qdot
figure(fig(3))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, eff_des(i,:), 'r--', 'linewidth',3);
  plot(time, eff_act(i,:), 'b-', 'linewidth',3);
  plot(time, grav(i,:), 'k-', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Jeff');
xlabel('Time (sec)','fontsize', 12);

# ee
figure(fig(4))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(ee_act(i,:) ,'b-','linewidth',3);
  plot(ee_des(i,:) ,'r--','linewidth',3);
  hold off

  grid on
end

title(tmp(1), 'EE Pos');
xlabel('Time (ms)','fontsize', 12);

# ee x_y
figure(fig(5))
hold on

plot(ee_act(1,:), ee_act(2,:), 'b-','linewidth',3);
plot(ee_des(1,:), ee_des(2,:), 'r--','linewidth',3);
axis([-0.2, 0.3, 0.4, 0.9], 'equal')

#grid on
#figure
#subplot(2,1,1)
#plot(err_sum(1,:));
#subplot(2,1,2)
#plot(err_sum(2,:));

#temp
figure(fig(6))
for i = 1:2
  tmp(i)=subplot(2,1,i);
  plot(time, core_temp(i,:), 'b-', 'linewidth', 3);
end
title(tmp(1), 'Temp');
xlabel('Time (sec)','fontsize', 12);

#efficiency
#figure(fig(6))
#for i = 1:2
#  tmp(i)=subplot(2,1,i);
#  plot(time, core_temp(i,:), 'b-', 'linewidth', 3);
#end
#title(tmp(1), 'Temp');
#xlabel('Time (sec)','fontsize', 12);