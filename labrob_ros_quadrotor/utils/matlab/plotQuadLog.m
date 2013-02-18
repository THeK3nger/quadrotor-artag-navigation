% Analysis file for quadrotor log given by gazebo

%% Load data
data = importdata('quad0_log.dat');

%% get vectors
time  = data.data(:, strcmp(data.colheaders,'time'));
x     = data.data(:, strcmp(data.colheaders,'x'));
y     = data.data(:, strcmp(data.colheaders,'y'));
z     = data.data(:, strcmp(data.colheaders,'z'));
roll  = data.data(:, strcmp(data.colheaders,'roll'));
pitch = data.data(:, strcmp(data.colheaders,'pitch'));
yaw   = data.data(:, strcmp(data.colheaders,'yaw'));
vx    = data.data(:, strcmp(data.colheaders,'vx'));
vy    = data.data(:, strcmp(data.colheaders,'vy'));
vz    = data.data(:, strcmp(data.colheaders,'vz'));
p     = data.data(:, strcmp(data.colheaders,'p'));
q     = data.data(:, strcmp(data.colheaders,'q'));
r     = data.data(:, strcmp(data.colheaders,'r'));

%% Plot data
figure;
plot(time, x, 'r', time, y, 'b', time, z, 'g')
legend('x','y','z')
title('position')
grid

figure;
plot(time, vx, 'r', time, vy, 'b', time, vz, 'g')
legend('V_x','V_y','V_z')
title('Linear velocities')
grid
