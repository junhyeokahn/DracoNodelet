function f_traj = fn_butterworth_filter(traj, num_sample, w_c, t_s)
    size_traj = size(traj);
    num_row = size_traj(1);
    num_col = size_traj(2);
    
    f_traj = zeros(size_traj);
    buffer = zeros(num_row, num_sample);
    
    for j = 1: num_col
        for i = num_sample-1:-1:1
            buffer(:, i+1) = buffer(:,i);

        end
        buffer(:, 1) = traj(:,j); 
        for k = 1: num_sample
            t = k*t_s;  
            f_traj(:, j) = f_traj(:, j) + ...
                           sqrt(2)/w_c * buffer(:,k)*exp(-1/sqrt(2) * t) * ...
                           ones(num_row,1) * sin(w_c/sqrt(2) *t)*t_s;
        end
    end

end