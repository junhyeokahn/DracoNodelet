function fig = fn_open_figure_window(num_figure)

original_x = 100;
position_x = original_x;
position_y = 700;
width = 560;
height = 420;

x_limit = 1800;

for i = 1:num_figure
  fig(i) = figure('position', [position_x, position_y, width, height]);
  position_x = position_x + width;
  if( position_x + width > x_limit)
    position_y = position_y -height;
    position_x = original_x;
  end
end

end