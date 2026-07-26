clc, clear variables
close all

set(groot, 'DefaultFigureWindowStyle', 'docked');

%% Channel indices
% Sent from C++:
% 1:  x_des                 [mm]
% 2:  y_des                 [mm] 
% 3:  x_meas cam            [mm]
% 4:  y_meas cam            [mm]
% 5:  x_hat                 [mm]
% 6:  y_hat                 [mm]
% 7:  vx_hat                [mm/s] 
% 8:  vy_hat                [mm/s]
% 9:  disturbance_x         [rad]
% 10: disturbance_y         [rad]
% 11:  controller output x  [rad]
% 12:  controller output y  [rad] 
% 13:  roll imu             [rad]
% 14:  pitch imu            [rad]

data_kreis = load("data_traj_kreis_50mm_0.35Hz.mat");
data_acht = load("data_traj_acht_50mm_25mm_0.3Hz.mat");

%% Resultate Kreis (Radius = 90mm, Frequenz = 0.5Hz)

time_kreis = data_kreis.data.time;
xd_o = data_kreis.data.values(:,1);
yd_o = data_kreis.data.values(:,2);
x_meas_o = data_kreis.data.values(:,3);
y_meas_o = data_kreis.data.values(:,4);
x_hat_o = data_kreis.data.values(:,5);
y_hat_o = data_kreis.data.values(:,6);
vx_hat_o = data_kreis.data.values(:,7);
vy_hat_o = data_kreis.data.values(:,8);

%% Resultate Kreis (Radius_x = 90mm, Radius_y = 45mm, Frequenz = 0.3Hz)
time_acht = data_acht.data.time;
xd_8 = data_acht.data.values(:,1);
yd_8 = data_acht.data.values(:,2);
x_meas_8 = data_acht.data.values(:,3);
y_meas_8 = data_acht.data.values(:,4);
x_hat_8 = data_acht.data.values(:,5);
y_hat_8 = data_acht.data.values(:,6);
vx_hat_8 = data_acht.data.values(:,7);
vy_hat_8 = data_acht.data.values(:,8);
roll_8 = data_acht.data.values(:,13);
pitch_8 = data_acht.data.values(:,14);

%% Plot Kreis (Circle Trajectory)
% xd vs x_meas / yd vs y_meas
figure(1)
subplot(2,1,1)
plot(time_kreis, xd_o, time_kreis, x_meas_o)
grid on
title('x_{d} vs x_{meas}')
xlabel('Time [s]')
ylabel('x [mm]')
legend('x_{d}', 'x_{meas}')
xlim([0 10])

subplot(2,1,2)
plot(time_kreis, yd_o, time_kreis, y_meas_o)
grid on
title('y_{d} vs y_{meas}')
xlabel('Time [s]')
ylabel('y [mm]')
legend('y_{d}', 'y_{meas}')
xlim([0 10])

% 2D-Plot
figure(2)
plot(xd_o, yd_o, x_meas_o, y_meas_o)
grid on
title('2D Trajectory')
xlabel('x [mm]')
ylabel('y [mm]')
legend('Desired trajectory', 'Measured trajectory')

%% Plot Acht (Figure-Eight Trajectory)
% xd vs x_meas / yd vs y_meas
figure(3)
subplot(2,1,1)
plot(time_acht, xd_8, time_acht, x_meas_8)
grid on
title('x_{d} vs x_{meas}')
xlabel('Time [s]')
ylabel('x [mm]')
legend('x_{d}', 'x_{meas}')
xlim([0 10])
ylim([-70 70])

subplot(2,1,2)
plot(time_acht, yd_8, time_acht, y_meas_8)
grid on
title('y_{d} vs y_{meas}')
xlabel('Time [s]')
ylabel('y [mm]')
legend('y_{d}', 'y_{meas}')
xlim([0 10])

% 2D-Plot
figure(4)
plot(xd_8, yd_8, x_meas_8, y_meas_8)
grid on
title('2D Trajectory')
xlabel('x [mm]')
ylabel('y [mm]')
legend('Desired trajectory', 'Measured trajectory')

figure(5)
plot(rad2deg(roll_8), rad2deg(pitch_8))
xlabel('roll [deg]')
ylabel('pitch [deg]')  