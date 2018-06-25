clear all
clc 
close all

%%
k = 5000000;
b = 5500;
m = 130;
num = [k];
den = [m, b, k];
sys1 = tf(num,den);

%%
path = '/home/apptronik/ros/ForceCtrl_Test/force_ctrl_test/exp_data_check';
fn_add_path('/home/apptronik/ros/apptronik_two_dt/data_manager/plotting_octave');

cmd = fn_read_file(path, 'received_joint_effort', 1);
%cmd = fn_read_file(path, 'received_motor_curr', 1);
time_ns = fn_read_file(path, 'time', 1);
current = fn_read_file(path, 'current_A', 1);
loadcell = fn_read_file(path, 'loadcell_N', 1);
motor_pos = fn_read_file(path, 'motor_pos_rad', 1);
rubber_deflection = fn_read_file(path, 'rubber_deflection_um', 1);
rubber_effort = fn_read_file(path, 'rubber_effort_N', 1);
t = time_ns;
offset = 0;

for i =1:length(time_ns)-1
  t(i) = time_ns(i) + offset;
  if time_ns(i+1) < time_ns(i)
    offset = offset + 1.e9;
  end
end

in_raw = cmd;
out_raw = rubber_effort;

t(end) = time_ns(end) + offset;
lin_t = linspace(t(1), t(end), length(t));

in = interp1(t, in_raw, lin_t);
out = interp1(t, out_raw, lin_t);

fin = fft(in);
fout = fft(out);

h = fout./fin;


for i = 1:length(h)
  mag(i) = norm(h(i));
  ang(i) = angle(h(i));
end

dt = lin_t(2) - lin_t(1);
x = linspace(0, 1/dt, length(lin_t));

figure
subplot(211)
semilogx(x, 20*log(mag),'b-','linewidth',3);
grid on
subplot(212)
semilogx(x, ang*180/pi,'b-','linewidth',3);
grid on

figure
%bode(sys1);
plot(cmd,'r','linewidth',3)
hold on
plot(rubber_effort,'b', 'linewidth',3)