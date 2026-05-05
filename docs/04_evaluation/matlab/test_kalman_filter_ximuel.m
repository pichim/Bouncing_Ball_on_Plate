clc, clear all
%% UART / SerialStream Aufnahme

port = '/dev/ttyUSB0'; % port = 'COM12';
baudrate = 2e6;

% Initialize the SerialStream object
try
    serialStream.reset();
    fprintf("Resetting existing serialStream object.\n")
catch exception
    serialStream = SerialStream(port, baudrate);
    fprintf("Creating new SerialStream object.\n")
end

% Starting the stream
serialStream.start()
while (serialStream.isBusy())
    pause(0.1);
end

% Accessing the data
try
    data = serialStream.getData();
catch exception
    fprintf("Data Stream not triggered.\n")
    return
end

% Save the data with timestamp
file_name = ['data_kalman_' datestr(now,'yyyymmdd_HHMMSS') '.mat'];
save(file_name, 'data');
fprintf("Saved data to: %s\n", file_name);

% Load the data
load(file_name)


%% Evaluate time

Ts = mean(diff(data.time));

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time [s]')
ylabel('dTime [mus]')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])


%% Index definition

% Data from C++ SerialStream:
%
%  0 Delta time in us              -> wird von SerialStream als data.time benutzt
%
% data.values:
%
%  1 acc x                         [m/s^2]
%  2 acc y                         [m/s^2]
%  3 acc z                         [m/s^2]
%  4 roll                          [rad]
%  5 pitch                         [rad]
%  6 x_meas camera                 [mm]
%  7 y_meas camera                 [mm]
%  8 x_hat Kalman                  [mm]
%  9 vx_hat Kalman                 [mm/s]
% 10 dx_hat Kalman disturbance     [rad]
% 11 y_hat Kalman                  [mm]
% 12 vy_hat Kalman                 [mm/s]
% 13 dy_hat Kalman disturbance     [rad]
% 14 camera update flag            [-]
% 15 kalman valid flag             [-]

if size(data.values,2) < 15
    error('Expected at least 15 channels in data.values, but got %d.', size(data.values,2));
end

ind.acc = 1:3;
ind.rpy = 4:5;

% Camera measurements
ind.x_meas = 6;
ind.y_meas = 7;

% Kalman x-axis
ind.x_hat  = 8;
ind.vx_hat = 9;
ind.dx_hat = 10;

% Kalman y-axis
ind.y_hat  = 11;
ind.vy_hat = 12;
ind.dy_hat = 13;

% Flags
ind.camera_update = 14;
ind.kalman_valid  = 15;


%% Extract data

% t = data.time;   % bleibt absichtlich inaktiv

acc = data.values(:, ind.acc);
rpy = data.values(:, ind.rpy);

x_meas = data.values(:, ind.x_meas);
y_meas = data.values(:, ind.y_meas);

x_hat  = data.values(:, ind.x_hat);
vx_hat = data.values(:, ind.vx_hat);
dx_hat = data.values(:, ind.dx_hat);

y_hat  = data.values(:, ind.y_hat);
vy_hat = data.values(:, ind.vy_hat);
dy_hat = data.values(:, ind.dy_hat);

camera_update = data.values(:, ind.camera_update);
kalman_valid  = data.values(:, ind.kalman_valid);

idx_update = camera_update > 0.5;
idx_valid  = kalman_valid > 0.5;


%% Basic plots: IMU data

figure(2)
plot(data.time, rpy * 180/pi, 'LineWidth', 1.2)
grid on
title('Roll and Pitch from IMU')
legend('roll','pitch', 'Location','best')
xlabel('Time [s]')
ylabel('Angle [deg]')


%% Basic plots: Acceleration data

figure(3)
plot(data.time, acc, 'LineWidth', 1.2)
grid on
title('Acceleration from IMU')
legend('acc x','acc y','acc z', 'Location','best')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')


%% Camera update and Kalman valid flags

figure(4)

subplot(2,1,1)
stairs(data.time, camera_update, 'LineWidth', 1.2)
grid on
title('Camera update flag')
ylabel('Update flag')

subplot(2,1,2)
stairs(data.time, kalman_valid, 'LineWidth', 1.2)
grid on
title('Kalman valid flag')
xlabel('Time [s]')
ylabel('Valid flag')

% Estimate camera update frequency
if data.time(end) > data.time(1)
    f_cam_est = sum(idx_update) / (data.time(end) - data.time(1));
    fprintf('\nEstimated camera update rate: %.2f Hz\n', f_cam_est);
end


%% Kalman validation: position camera vs. Kalman

figure(5)

subplot(2,1,1)
plot(data.time(idx_update), x_meas(idx_update), 'g.', 'MarkerSize', 8)
hold on
plot(data.time, x_hat, 'r', 'LineWidth', 1.3)
grid on
title('Kalman validation: x position')
ylabel('x [mm]')
legend('x camera update','x Kalman', 'Location','best')

subplot(2,1,2)
plot(data.time(idx_update), y_meas(idx_update), 'g.', 'MarkerSize', 8)
hold on
plot(data.time, y_hat, 'r', 'LineWidth', 1.3)
grid on
title('Kalman validation: y position')
xlabel('Time [s]')
ylabel('y [mm]')
legend('y camera update','y Kalman', 'Location','best')


%% Kalman validation: estimated velocity

figure(6)
plot(data.time, vx_hat, 'LineWidth', 1.2)
hold on
plot(data.time, vy_hat, 'LineWidth', 1.2)
grid on
title('Estimated ball velocity')
xlabel('Time [s]')
ylabel('Velocity [mm/s]')
legend('vx hat','vy hat', 'Location','best')


%% Kalman validation: estimated input disturbance

figure(7)
plot(data.time, rad2deg(dx_hat), 'LineWidth', 1.2)
hold on
plot(data.time, rad2deg(dy_hat), 'LineWidth', 1.2)
grid on
title('Estimated input disturbance')
xlabel('Time [s]')
ylabel('Disturbance [deg]')
legend('dx hat','dy hat', 'Location','best')


%% Combined Kalman overview

figure(8)
tiledlayout(4,1)

nexttile
plot(data.time(idx_update), x_meas(idx_update), 'g.', 'MarkerSize', 8)
hold on
plot(data.time, x_hat, 'r', 'LineWidth', 1.3)
grid on
ylabel('x [mm]')
title('x camera vs. x Kalman')
legend('x camera','x hat', 'Location','best')

nexttile
plot(data.time(idx_update), y_meas(idx_update), 'g.', 'MarkerSize', 8)
hold on
plot(data.time, y_hat, 'r', 'LineWidth', 1.3)
grid on
ylabel('y [mm]')
title('y camera vs. y Kalman')
legend('y camera','y hat', 'Location','best')

nexttile
plot(data.time, vx_hat, 'LineWidth', 1.2)
hold on
plot(data.time, vy_hat, 'LineWidth', 1.2)
grid on
ylabel('v [mm/s]')
title('Estimated velocity')
legend('vx hat','vy hat', 'Location','best')

nexttile
plot(data.time, rad2deg(dx_hat), 'LineWidth', 1.2)
hold on
plot(data.time, rad2deg(dy_hat), 'LineWidth', 1.2)
grid on
ylabel('d [deg]')
xlabel('Time [s]')
title('Estimated disturbance')
legend('dx hat','dy hat', 'Location','best')


%% Numeric checks for Kalman validation

idx_eval_update = idx_valid & idx_update;
idx_eval_valid  = idx_valid;

fprintf('\n--- Kalman validation statistics ---\n')

if sum(idx_eval_update) > 10
    std_x_meas = std(x_meas(idx_eval_update));
    std_x_hat  = std(x_hat(idx_eval_update));

    std_y_meas = std(y_meas(idx_eval_update));
    std_y_hat  = std(y_hat(idx_eval_update));

    fprintf('std x camera: %.3f mm\n', std_x_meas)
    fprintf('std x Kalman: %.3f mm\n', std_x_hat)
    fprintf('std y camera: %.3f mm\n', std_y_meas)
    fprintf('std y Kalman: %.3f mm\n', std_y_hat)
else
    fprintf('Not enough camera update samples for position noise statistics.\n')
end

if sum(idx_eval_valid) > 10
    fprintf('mean abs vx_hat: %.3f mm/s\n', mean(abs(vx_hat(idx_eval_valid))))
    fprintf('mean abs vy_hat: %.3f mm/s\n', mean(abs(vy_hat(idx_eval_valid))))
    fprintf('max abs dx_hat: %.3f deg\n', max(abs(rad2deg(dx_hat(idx_eval_valid)))))
    fprintf('max abs dy_hat: %.3f deg\n', max(abs(rad2deg(dy_hat(idx_eval_valid)))))
else
    fprintf('Not enough valid Kalman samples for velocity/disturbance statistics.\n')
end