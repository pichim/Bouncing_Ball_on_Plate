clc, clear variables
close all

set(groot, 'DefaultFigureWindowStyle', 'docked');

%% Channel indices
data_raw = load("acc_z_up.mat")
data_calib = load("messung_mit_imu_calib_ruhe_0Volt.mat")
data_calib2 = load("data_still_plate_calibrated_roll_pitch_measurement.mat")

% Sent from C++:
% 1:  servo1 command
% 2:  servo2 command
% 3:  servo3 command
% 4:  gyro x [rad/s]
% 5:  gyro y [rad/s]
% 6:  gyro z [rad/s]
% 7:  acc x [m/s^2]
% 8:  acc y [m/s^2]
% 9:  acc z [m/s^2]
% 10: roll [rad]
% 11: pitch [rad]
% 12: yaw [rad]

time_raw = data_raw.data.time;
gyro_raw_x = data_raw.data.values(:,1);
gyro_raw_y = data_raw.data.values(:,2);
gyro_raw_z = data_raw.data.values(:,3); 

time_calib = data_calib.data.time;
gyro_calib_x = data_calib.data.values(:,4);
gyro_calib_y = data_calib.data.values(:,5);
gyro_calib_z = data_calib.data.values(:,6);

time_calib2 = data_calib2.data.time;
roll_calib2 = data_calib2.data.values(:,1);
pitch_calib2 = data_calib2.data.values(:,2);

figure(1)
sgtitle('Gyroscope Comparison: Raw vs. Calibrated')
subplot(1,2,1)
plot(time_raw, gyro_raw_x, time_raw, gyro_raw_y, time_raw, gyro_raw_z)
xlim([0 10])
ylim([-0.15 0.16])
title('Gyro - Raw Data')
xlabel('Time [s]')
ylabel('Angular Velocity [rad/s]')
grid on
legend('Gyro X', 'Gyro Y', 'Gyro Z')

subplot(1,2,2)
plot(time_calib, gyro_calib_x, time_calib, gyro_calib_y, time_calib, gyro_calib_z)
xlim([0 10])
ylim([-0.15 0.16])
title('Gyro - Calibrated')
xlabel('Time [s]')
ylabel('Angular Velocity [rad/s]')
grid on
legend('Gyro X', 'Gyro Y', 'Gyro Z')

figure(2)
plot(time_calib2, roll_calib2, time_calib2, pitch_calib2, 'LineWidth', 2)
xlim([0 10])
ylim([-0.01 0])
title('Roll & Pitch - Calibrated')
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('Roll', 'Pitch')
grid on