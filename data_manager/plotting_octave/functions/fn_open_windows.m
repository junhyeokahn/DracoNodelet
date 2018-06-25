function fig = fn_open_windows(num_window)

original_x = 0;
position_x = original_x;
position_y = 500;
width = 560;
height = 460;
increase_x = 490;
increase_y = 2*460;

x_limit = 1800;

for i = 1:num_window
  fig(i) = figure('position', [position_x, position_y, width, height]);

  position_x = position_x + increase_x;
  if( position_x + width > x_limit)
    position_y = position_y - increase_y;
    position_x = original_x;
  end

end

end