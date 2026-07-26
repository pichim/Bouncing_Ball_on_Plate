clc, clear variables
close all

set(groot, 'DefaultFigureWindowStyle', 'docked');

%% Channel indices
% Sent from C++:
% 1:  Roll soll     [rad]
% 2:  Pitch soll    [rad] 
% 3:  Höhe soll     [mm]
% 4:  Roll IMU      [rad]
% 5:  Pitch IMU     [rad]

%% Messung Roll (um x-Achse)
data_roll_1 = load("data_roll_new.mat")
data_roll_2 = load("data_roll_new_2.mat")

% Messung 1
time_roll_1 = data_roll_1.data.time;
roll_soll_1 = rad2deg(data_roll_1.data.values(:,1));
roll_imu_1 = rad2deg(data_roll_1.data.values(:,4));
pitch_soll_1 = rad2deg(data_roll_1.data.values(:,2));
pitch_imu_1 = rad2deg(data_roll_1.data.values(:,5));

% Messung 2 [für Doku]
time_roll_2 = data_roll_2.data.time;
roll_soll_2 = rad2deg(data_roll_2.data.values(:,1));
roll_imu_2 = rad2deg(data_roll_2.data.values(:,4));
pitch_soll_2 = rad2deg(data_roll_2.data.values(:,2));
pitch_imu_2 = rad2deg(data_roll_2.data.values(:,5));

error_roll_2 = -roll_soll_2 - roll_imu_2;

%% Messung Pitch (um y-Achse)
data_pitch_1 = load("data_pitch_new.mat")
data_pitch_2 = load("data_pitch_new_2.mat")

% Messung 1
time_pitch_3 = data_pitch_1.data.time;
roll_soll_3 = rad2deg(data_pitch_1.data.values(:,1));
roll_imu_3 = rad2deg(data_pitch_1.data.values(:,4));
pitch_soll_3 = rad2deg(data_pitch_1.data.values(:,2));
pitch_imu_3 = rad2deg(data_pitch_1.data.values(:,5));

% Messung 2 [Für Doku]
time_pitch_4 = data_pitch_2.data.time;
roll_soll_4 = rad2deg(data_pitch_2.data.values(:,1));
roll_imu_4 = rad2deg(data_pitch_2.data.values(:,4));
pitch_soll_4 = rad2deg(data_pitch_2.data.values(:,2));
pitch_imu_4 = rad2deg(data_pitch_2.data.values(:,5));

error_pitch_4 = -pitch_soll_4 - pitch_imu_4;

%% Roll Plots 
figure(1)

% subplot(2,1,1)
% plot(time_roll_1, -roll_soll_1, 'LineWidth', 1.2)
% hold on
% plot(time_roll_1, roll_imu_1, 'LineWidth', 1.2)
% grid on
% title('Measurement 1')
% xlabel('Time [s]')
% ylabel('Roll angle [°]')
% legend('Commanded roll angle', 'Measured roll angle (IMU)', 'Location', 'best')
% xlim([2.5 42.5])

plot(time_roll_2, -roll_soll_2, 'LineWidth', 1.2)
hold on
plot(time_roll_2, roll_imu_2, 'LineWidth', 1.2)
grid on
title('Roll angle results at different commanded roll angles')
xlabel('Time [s]')
ylabel('Roll angle [°]')
legend('Commanded roll angle', 'Measured roll angle (IMU)', 'Location', 'best')
xlim([2.5 42.5])

% subplot(2,1,2)
% plot(time_roll_2, error_roll_2, 'LineWidth', 1.2)
% grid on
% title('Roll error')
% xlabel('Time [s]')
% ylabel('Error roll angle [°]')
% xlim([2.5 42.5])
% ylim([-5 5])

%% Pitch Plots 
figure(2)

% subplot(2,1,1)
% plot(time_pitch_3, -pitch_soll_3, 'LineWidth', 1.2)
% hold on
% plot(time_pitch_3, pitch_imu_3, 'LineWidth', 1.2)
% grid on
% title('Measurement 1')
% xlabel('Time [s]')
% ylabel('Pitch angle [°]')
% legend('Commanded pitch angle', 'Measured pitch angle (IMU)', 'Location', 'best')
% xlim([2.5 42.5])

plot(time_pitch_4, -pitch_soll_4, 'LineWidth', 1.2)
hold on
plot(time_pitch_4, pitch_imu_4, 'LineWidth', 1.2)
grid on
title('Pitch angle results at different commanded pitch angles')
xlabel('Time [s]')
ylabel('Pitch angle [°]')
legend('Commanded pitch angle', 'Measured pitch angle (IMU)', 'Location', 'best')
xlim([2.5 42.5])

% subplot(2,1,2)
% plot(time_pitch_4, error_pitch_4, 'LineWidth', 1.2)
% grid on
% title('Pitch error')
% xlabel('Time [s]')
% ylabel('Error pitch angle [°]')
% xlim([2.5 42.5])
% ylim([-5 5])

%% Error mean berechnung von ChatGPT

settling_time = 2.0;   % [s] ignore after each command step
end_margin    = 0.2;   % [s] ignore shortly before next step
step_threshold = 0.5;  % [deg] threshold to detect command steps

% Sign convention used in the plots above
roll_cmd  = -roll_soll_2;
roll_meas =  roll_imu_2;

pitch_cmd  = -pitch_soll_4;
pitch_meas =  pitch_imu_4;

% Calculate steady-state values
[roll_cmd_level, roll_meas_mean, roll_err_mean, roll_err_std] = ...
    getSteadyStateValues(time_roll_2, roll_cmd, roll_meas, settling_time, end_margin, step_threshold);

[pitch_cmd_level, pitch_meas_mean, pitch_err_mean, pitch_err_std] = ...
    getSteadyStateValues(time_pitch_4, pitch_cmd, pitch_meas, settling_time, end_margin, step_threshold);

%% Plots error roll und pitch  
figure(3)
plot(roll_cmd_level, roll_err_mean, 'o', 'LineStyle', 'none', 'LineWidth', 1.2, 'MarkerSize', 7)
hold on
grid on
yline(0, '--')

% Linear fit
p_roll = polyfit(roll_cmd_level, roll_err_mean, 1);

x_fit_roll = linspace(min(roll_cmd_level), max(roll_cmd_level), 100);
y_fit_roll = polyval(p_roll, x_fit_roll);

plot(x_fit_roll, y_fit_roll, 'LineWidth', 1.0)

title('Roll steady-state error')
xlabel('Commanded roll angle [°]')
ylabel('Mean roll error [°]')
legend('Mean steady-state error', 'Zero error', 'Linear fit', 'Location', 'best')

fprintf('Roll error fit: error = %.4f * command + %.4f deg\n', p_roll(1), p_roll(2));

figure(4)
plot(pitch_cmd_level, pitch_err_mean, 'o', 'LineStyle', 'none', 'LineWidth', 1.2, 'MarkerSize', 7)
hold on
grid on
yline(0, '--')

% Linear fit
p_pitch = polyfit(pitch_cmd_level, pitch_err_mean, 1);

x_fit_pitch = linspace(min(pitch_cmd_level), max(pitch_cmd_level), 100);
y_fit_pitch = polyval(p_pitch, x_fit_pitch);

plot(x_fit_pitch, y_fit_pitch, 'LineWidth', 1.0)

title('Pitch steady-state error')
xlabel('Commanded pitch angle [°]')
ylabel('Mean pitch error [°]')
legend('Mean steady-state error', 'Zero error', 'Linear fit', 'Location', 'best')

fprintf('Pitch error fit: error = %.4f * command + %.4f deg\n', p_pitch(1), p_pitch(2));

%% Print results
fprintf('\n--- Roll validation ---\n')
fprintf('Measured roll = %.3f * commanded roll + %.3f deg\n', p_roll(1), p_roll(2));
fprintf('Mean abs roll error = %.3f deg\n', mean(abs(roll_err_mean)));
fprintf('Max abs roll error  = %.3f deg\n', max(abs(roll_err_mean)));

fprintf('\n--- Pitch validation ---\n')
fprintf('Measured pitch = %.3f * commanded pitch + %.3f deg\n', p_pitch(1), p_pitch(2));
fprintf('Mean abs pitch error = %.3f deg\n', mean(abs(pitch_err_mean)));
fprintf('Max abs pitch error  = %.3f deg\n', max(abs(pitch_err_mean)));

%% Hilfsfunktion von ChatGPT
function [cmd_level, meas_mean, err_mean, err_std] = getSteadyStateValues(time, cmd, meas, settling_time, end_margin, step_threshold)

    err = cmd - meas;

    % Find step changes in commanded angle
    step_idx = [1; find(abs(diff(cmd)) > step_threshold) + 1; length(time) + 1];

    cmd_level = [];
    meas_mean = [];
    err_mean = [];
    err_std = [];

    for k = 1:length(step_idx)-1

        i1 = step_idx(k);
        i2 = step_idx(k+1) - 1;

        idx = i1:i2;

        % Only use steady-state part
        valid = idx(time(idx) >= time(i1) + settling_time & ...
                    time(idx) <= time(i2) - end_margin);

        if length(valid) > 10
            cmd_level(end+1) = mean(cmd(valid));
            meas_mean(end+1) = mean(meas(valid));
            err_mean(end+1) = mean(err(valid));
            err_std(end+1) = std(err(valid));
        end
    end
end