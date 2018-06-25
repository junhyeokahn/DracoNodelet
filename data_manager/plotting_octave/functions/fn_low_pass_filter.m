function f_traj = fn_low_pass_filter(traj, w_c, t_s )
    Lpf_in_prev = zeros(1,2);
    Lpf_out_prev = zeros(1,2);

    den = 2500*t_s*t_s*w_c*w_c  + 7071*t_s*w_c + 10000;

    Lpf_in1 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_in2 = 5000*t_s*t_s*w_c*w_c / den;
    Lpf_in3 = 2500*t_s*t_s*w_c*w_c / den;
    Lpf_out1 = -(5000*t_s*t_s*w_c*w_c  - 20000) / den;
    Lpf_out2 = -(2500*t_s*t_s*w_c*w_c  - 7071*t_s*w_c + 10000) / den;
    
    size_traj = size(traj);
    f_traj = zeros(size_traj);
    num_row = size_traj(1);
    num_col = size_traj(2);

    for j = 1:num_row
        for i = 1: num_col
            lpf_in = traj(j,i);
            lpf_out = Lpf_in1*lpf_in + Lpf_in2*Lpf_in_prev(1) + Lpf_in3*Lpf_in_prev(2) + ... %input component
                      Lpf_out1*Lpf_out_prev(1) + Lpf_out2*Lpf_out_prev(2); %output component
            Lpf_in_prev(2) = Lpf_in_prev(1);
            Lpf_in_prev(1) = lpf_in;
            Lpf_out_prev(2) = Lpf_out_prev(1);
            Lpf_out_prev(1) = lpf_out;

            f_traj(j,i) = lpf_out;
        end
    end
end