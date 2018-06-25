clear all
clc 
close all

%% 
path = '/home/apptronik/ros/apptronik_two_dt/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

actuator_effort = fn_read_file(path, 'actuator_effort', 2);
joint_effort = fn_read_file(path, 'joint_effort', 2);
motor_effort = fn_read_file(path, 'motor_effort', 2);

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


%% Draw Figure

figure(fig(1))

for i = 1:2
  tmp(i) = subplot(2, 1, i);
  hold on
  X=[ones(length(motor_effort(i,:)), 1) (motor_effort(i,:))'];
  theta = (pinv(X'*X))*X'*(joint_effort(i,:)')
  plot(motor_effort(i,:), joint_effort(i,:), '*');
  hold on
  plot(motor_effort(i,:), theta(1) * motor_effort(i,:) + theta(2), 'r-', 'linewidth',3)
  hold off
  ylabel('joint effort(Nm)','fontsize',12);
  xlabel('motor effort(A)','fontsize', 12);
  
  set(gca, 'fontsize', 12);
  grid on
end

figure(fig(2))
for i = 1:2
  tmp(i) = subplot(2, 1, i);
  hold on
  X=[ones(length(motor_effort(i,:)), 1) (motor_effort(i,:))'];
  theta = (pinv(X'*X))*X'*(actuator_effort(i,:)')
  plot(motor_effort(i,:), actuator_effort(i,:), '*');
  hold on
  plot(motor_effort(i,:), theta(1) * motor_effort(i,:) + theta(2), 'r-', 'linewidth',3)
  hold off
  ylabel('actuator effort(N)','fontsize',12);
  xlabel('motor effort(A)','fontsize', 12);
  
  set(gca, 'fontsize', 12);
  grid on
end
