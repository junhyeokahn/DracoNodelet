clear all
clc 
close all

%% 
path = '/home/apptronik/ros/DracoNodelet/data_manager/experiment_data_check';
fn_add_path('/home/apptronik/ros/DracoNodelet/data_manager/plotting_octave');


time = fn_read_file(path, 'time', 1);
JPosAct = fn_read_file(path, 'JPosAct', 2);
JVelAct = fn_read_file(path, 'JVelAct', 2);
JEffAct = fn_read_file(path, 'JEffAct', 2);
JPosDes = fn_read_file(path, 'JPosDes', 2);
JVelDes = fn_read_file(path, 'JVelDes', 2);
JEffDes = fn_read_file(path, 'JEffDes', 2);

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 400;
increase_x = 560;
increase_y = 470;

x_limit = 1800;

num_figure = 3;
for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end
end


%% Draw Figure
endIdx = length(time) - 2000;
#q
figure(fig(1))

for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time(1:endIdx), JPosDes(i,1:endIdx) ,'r-','linewidth',3);
  plot(time(1:endIdx), JPosAct(i,1:endIdx), 'b-', 'linewidth',3);
%  if (i == 1)
%    plot(time(1:endIdx), KneeMJPos(i,1:endIdx) ,'g-','linewidth',3);
%  end
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Joint Position');
xlabel('Time (sec)','fontsize', 12);

# qdot
figure(fig(2))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time(1:endIdx), JVelAct(i,1:endIdx),'b-','linewidth',3);
  hold on
  plot(time(1:endIdx), JVelDes(i,1:endIdx), 'r-', 'linewidth',3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Joint Vel');
xlabel('Time (sec)','fontsize', 12);

# jtrq
figure(fig(3))
for i = 1:2
  tmp(i) = subplot(2,1,i);
  hold on
  plot(time(1:endIdx), JEffDes(i,1:endIdx), 'r-', 'linewidth',3);
  plot(time(1:endIdx), JEffAct(i,1:endIdx), 'b-', 'linewidth', 3);
  hold off

  set(gca, 'fontsize',12);
  grid on
end

title(tmp(1), 'Jeff');
xlabel('Time (sec)','fontsize', 12);