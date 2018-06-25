clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

time = fn_read_file(path, 'curr_time', 1);
jpos_act = fn_read_file(path, 'joint_pos', 2);
jvel_act = fn_read_file(path, 'joint_vel', 2);
motor_jpos_act = fn_read_file(path, 'motor_joint_pos', 2);
motor_jvel_act = fn_read_file(path, 'motor_joint_vel', 2);
jeff_act = fn_read_file(path, 'jeff', 2);
jpos_des = fn_read_file(path, 'jpos_cmd', 2);
jvel_des = fn_read_file(path, 'jvel_cmd', 2);
jeff_des = fn_read_file(path, 'jeff_cmd', 2);
m_eff = fn_read_file(path, 'motor_current', 2);
a_eff = fn_read_file(path, 'actuator_eff', 2);
ee_act = fn_read_file(path, 'ee_act', 2);
ee_des = fn_read_file(path, 'ee_des',2);
tempe = fn_read_file(path, 'temperature', 2);
bus_v = fn_read_file(path, 'bus_v',2);
bus_a = fn_read_file(path, 'bus_a',2);
avel_act = fn_read_file(path, 'act_vel',2);

%% Data truncate
end_idx = length(time)-500;
st_idx =  1%end_idx - 1000;
time = time(st_idx:end_idx);
jpos_act = jpos_act(:, st_idx:end_idx);
jvel_act = jvel_act(:, st_idx:end_idx);
motor_jpos_act = motor_jpos_act(:, st_idx:end_idx);
motor_jvel_act = motor_jvel_act(:, st_idx:end_idx);
jeff_act = jeff_act(:, st_idx:end_idx);
jpos_des = jpos_des(:, st_idx:end_idx);
jvel_des = jvel_des(:, st_idx:end_idx);
jeff_des = jeff_des(:, st_idx:end_idx);
m_eff = m_eff(:, st_idx:end_idx);
a_eff = a_eff(:, st_idx:end_idx);

ee_act = ee_act(:, st_idx:end_idx);
ee_des = ee_des(:, st_idx:end_idx);
tempe = tempe(:, st_idx:end_idx);
bus_v = bus_v(:, st_idx:end_idx);
bus_a = bus_a(:, st_idx:end_idx);
avel_act = avel_act(:, st_idx:end_idx);



original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;
x_limit = 1800;
num_figure = 9;

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
fig_num = 1;
figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, jpos_act(i,:) ,'b-','linewidth',3);
  plot(time, jpos_des(i,:), 'r--', 'linewidth',3);
  plot(time, motor_jpos_act(i, :), 'g-', 'linewidth',3);
  hold off
  set(gca, 'fontsize',12);
  grid on
  axis tight
end
fig_num = fig_num + 1;
title(tmp(1), 'Jpos');
xlabel('Time (sec)','fontsize', 12);

# qdot
figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, jvel_act(i,:),'b-','linewidth',3);
  plot(time, jvel_des(i,:), 'r--', 'linewidth',3);
  plot(time, motor_jvel_act(i, :), 'g-', 'linewidth',3);
  hold off
  set(gca, 'fontsize',12);
  grid on
    axis tight

end
fig_num = fig_num + 1;
title(tmp(1), 'Jvel');
xlabel('Time (sec)','fontsize', 12);

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, jeff_des(i,:), 'r--', 'linewidth',3);
  plot(time, jeff_act(i,:), 'b-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
title(tmp(1), 'Jeff');
xlabel('Time (sec)','fontsize', 12);

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  plot(time, avel_act(i,:),'b-','linewidth',3);
  line([time(1), time(end)], [0.298, 0.298]);
  line([time(1), time(end)], [-0.298, -0.298]);
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
title(tmp(1), 'Avel');
xlabel('Time (sec)','fontsize', 12);

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, m_eff(i,:), 'b-', 'linewidth', 3);
  plot(time, bus_v(i,:), 'r-', 'linewidth', 3);
    plot(time, bus_a(i,:), 'g-', 'linewidth', 3);
      plot(time, bus_v(i,:).*bus_a(i,:), 'm-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Current');

% Temperature
figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, tempe(i,:), 'b-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'temperature');

# ee pos cartesian
figure(fig(fig_num))
st_idx = 100;
end_idx = length(time);
hold on
plot(ee_act(1, st_idx:end_idx), ee_act(2, st_idx:end_idx), 'b','linewidth', 3);
plot(ee_des(1, st_idx:end_idx), ee_des(2, st_idx:end_idx), 'r--', 'linewidth', 3);
hold off
ylim([0.5, 0.95])
xlim([-0.1, 0.1])
xlabel('x (m)','fontsize', 12);
ylabel('y (m)','fontsize', 12);
fig_num = fig_num + 1;

# ee pos
figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, ee_act(i,:), 'b-', 'linewidth', 3);
  plot(time, ee_des(i,:), 'r--', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
  axis tight
end
fig_num = fig_num + 1;
title(tmp(1), 'EE pos');
xlabel('Time (sec)','fontsize', 12);

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, a_eff(i,:), 'b-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'aeff');

# efficiency
figure(fig(fig_num))
e_pow = bus_a .* bus_v;
m_pow = jeff_act .* motor_jvel_act;
eff = m_pow ./ e_pow;
for i = 1:2
  tmp(i) = subplot(2,1,i);
  plot(time, eff(i,:),'b-','linewidth',3);
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
title(tmp(1), 'Efficiency');
xlabel('Time (sec)','fontsize', 12);

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, m_pow(i,:), 'b-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Mech Power');

figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, motor_jvel_act(i,:),'b-','linewidth',3);
  hold on
  plot(time, jvel_des(i,:), 'r--', 'linewidth',3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
title(tmp(1), 'MJvel');
xlabel('Time (sec)','fontsize', 12);



# motor effort
figure(fig(fig_num))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time, m_eff(i,:), 'b-', 'linewidth', 3);
  hold off
  set(gca, 'fontsize',12);
  grid on
end
fig_num = fig_num + 1;
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Mot eff');


