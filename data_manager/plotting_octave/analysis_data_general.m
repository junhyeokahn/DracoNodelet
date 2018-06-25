clear all
clc 
close all

%% 
%path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
% NAS
path = '/home/apptronik/MyCloud/Apptronik/LatestData/';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');


time = fn_read_file(path, 'curr_time', 1);
config = fn_read_file(path, 'config', 2);
qdot = fn_read_file(path, 'qdot', 2);
config_des = fn_read_file(path, 'jpos_cmd', 2);
qdot_des = fn_read_file(path, 'jvel_cmd', 2);
jeff_des = fn_read_file(path, 'jeff_cmd', 2);
jeff = fn_read_file(path, 'jeff', 2);
err_sum = fn_read_file(path, 'err_sum',2);
motor_eff = fn_read_file(path, 'motor_effort', 2);
ee_act = fn_read_file(path, 'ee_act', 2);
ee_des = fn_read_file(path, 'ee_des',2);
tempe = fn_read_file(path, 'temperature', 2);

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 7;
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
  plot(time, qdot(i,:),'b-','linewidth',3);
  hold on
  plot(time, qdot_des(i,:), 'r--', 'linewidth',3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Qdot');
xlabel('Time (sec)','fontsize', 12);


for i = 1:length(jeff_des)
for j = 1:2
value = jeff_des(j, i);
max = 100;
if(value > max)
jeff_des(j, i) = max;
elseif (value < -max)
jeff_des(j, i) = -max;
end

end
end
# qdot
figure(fig(3))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, jeff_des(i,:), 'r--', 'linewidth',3);
  plot(time, jeff(i,:), 'b-', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Jeff');
xlabel('Time (sec)','fontsize', 12);

# ee pos
figure(fig(4))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, ee_act(i,:), 'b-', 'linewidth', 3);
  plot(time, ee_des(i,:), 'r--', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'EE pos');
xlabel('Time (sec)','fontsize', 12);

figure(fig(5))
st_idx = 100;
end_idx = length(time);
hold on
plot(ee_act(1, st_idx:end_idx), ee_act(2, st_idx:end_idx), 'b','linewidth', 3);
plot(ee_des(1, st_idx:end_idx), ee_des(2, st_idx:end_idx), 'r--', 'linewidth', 3);
hold off
ylim([0.5, 0.9])
xlim([-0.2, 0.2])
# motor effort
figure(fig(6))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, motor_eff(i,:), 'b-', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Mot eff');


figure(fig(7))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, tempe(i,:), 'b-', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'temperature');