clc, clear all
%%

port = '/dev/ttyUSB0'; % port = 'COM12';
baudrate = 2e6;


% Initialize the SerialStream object
try
    serialStream.reset();
    fprintf("Resetting existing serialStream object.\n")
catch exception
    serialStream = SerialStream(port, baudrate);
    fprintf("Creating new serialStream object.\n")
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

% Save the data
file_name = 'data_00.mat';
save(file_name, 'data');

% Load the data
load(file_name)


%% Evaluate time

Ts = mean(diff(data.time));

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus,2.1 ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])


%% Evaluate the data

% ind.servo_commands = 1;
% 
% figure(2)
% plot(data.time, data.values(:, ind.servo_commands))

ind.servo_commands = 1:3;
ind.gyro = 4:6;
ind.acc = 7:9;
ind.rpy = 10:12;

%% Figure 2: Servo Commands
figure(2)
plot(data.time, data.values(:, ind.servo_commands), 'LineWidth', 1.5)
grid on
xlabel('Zeit (s)')
ylabel('Servo Kommando')
title('Servo Befehle über die Zeit')
legend('Servo 1', 'Servo 2', 'Servo 3', 'Location', 'best')

%% Figure 3: Rohdaten (Gyro, Acc, RPY)
figure(3)

subplot(311)
plot(data.time, data.values(:, ind.gyro), 'LineWidth', 1.5)
grid on
ylabel('Winkelgeschw. (rad/s)')
title('Gyroskop Daten')
legend('Gyro X', 'Gyro Y', 'Gyro Z', 'Location', 'best')

subplot(312)
plot(data.time, data.values(:, ind.acc), 'LineWidth', 1.5)
grid on
ylabel('Beschl. (m/s^2)') % Falls dein Sensor in 'g' misst, hier anpassen!
title('Winkelbeschl. Daten')
legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'best')

subplot(313)
plot(data.time, data.values(:, ind.rpy), 'LineWidth', 1.5)
grid on
xlabel('Zeit (s)')
ylabel('Winkel (rad)')
title('Orientierung (Roll, Pitch, Yaw)')
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best')

%% Figure 31: Einzelvergleich Servo 3 vs Roll
figure(31) 
plot(data.time, data.values(:, [3, 10]), 'LineWidth', 1.5)
grid on
xlabel('Zeit (s)')
ylabel('Winkel (rad)')
title('Servo Command vs. Roll-Winkel')
legend('Servo Command', 'Roll', 'Location', 'best')

%% Figure 4: Mahony Filter Vergleich (C++ vs MATLAB)
addpath mahony/
para.kp = 0.1592 * 2.0 * pi;
para.ki = 0.0;
rpy0 = data.values(1, ind.rpy);
quat0 = rpy2quat(rpy0).';
[quatRP , biasRP] = mahonyRP(data.values(:,ind.gyro ), data.values(:,ind.acc), para, Ts, quat0);
rpyRP  = quat2rpy(quatRP);

figure(4)
plot(data.time, [data.values(:, ind.rpy), rpyRP], 'LineWidth', 1.5)
grid on 
xlabel('Zeit (s)')
ylabel('Winkel (rad)')
title('Mahony Filter Vergleich: C++ vs. MATLAB')
legend('Roll (C++)', 'Pitch (C++)', 'Yaw (C++)', 'Roll (MATLAB)', 'Pitch (MATLAB)', 'Yaw (MATLAB)', 'Location', 'best')

%% Figure 5: Servo Frequency Response (Chirp → Gyro Z)
% --- Signale ---
t = data.time;
Ts = mean(diff(t));          % Samplingzeit
fs = 1/Ts;                   % Samplingfrequenz
fmin = 1;
fmax = 10;
% u = (data.values(:,1)) * pi * 110 / 180;   % Servo-Chirp Input (0-1) normiert auf rad
% y = detrend(data.values(:,11));   % Gyro Z (rad/s) 6  // Yaw (rad) 12

u = data.values(:,3);    % Servo-Chirp Input (0-rad)
y = data.values(:,10);   % roll (rad)
%u = u - mean(u);
%y = y - mean(y);

figure(5)
plot(t, [u, y], 'LineWidth', 1.5)
grid on
xlabel('Zeit (s)')
ylabel('Amplitude (rad)')
title('System-Ein- und Ausgang für Frequenzanalyse')
legend('Input u (Servo Chirp)', 'Output y (Roll)', 'Location', 'best')

Tend = t(end);
Nest = round(5 / Ts);
win = hann(Nest);
noverlap = round(0.5 * Nest);

[gest, freq] = tfestimate(u, y, win, noverlap, [], 1/Ts);
cest = mscohere(u, y, win, noverlap, [], 1/Ts);

Gest = frd(gest, freq, Ts, 'Units', 'Hz');

Cest = frd(cest, freq, Ts, 'Units', 'Hz');

%% Figure 6: Bode Diagramm Servo
figure(6)
bode(Gest)
grid on
ylim([-360, 10]) % Wirkt auf die Phase
xlim([1, 1000])
% sgtitle (Super-Title) setzt den Titel mittig über beide Subplots!
title('Bode Diagramm Servo', 'FontWeight', 'bold')

%% Figure 7: Kohärenz
figure(7)
bodemag(Cest)
grid on
ylim([-20, 10])
xlim([1, 1000])
% Hier reicht ein normales title, da es nur ein Graph ist
title('Kohärenz Bode Diagramm Servo')

% % --- Frequenzgang-Schätzung ---
% N = length(u);
% window = hann(floor(N/4));
% noverlap = floor(length(window)/2);
% nfft = length(window);
% 
% [H,f] = tfestimate(u, y, window, noverlap, nfft, fs);
% 
% 
% idx = (f >= fmin) & (f <= fmax);
% 
% % --- Bode Plot ---
% figure(6)
% subplot(2,1,1)
% semilogx(f(idx),20*log10(abs(H(idx))),'LineWidth',1.5)
% grid on
% ylabel('Magnitude (dB)')
% title('Servo Frequency Response')
% 
% subplot(2,1,2)
% semilogx(f(idx),angle(H(idx))*180/pi,'LineWidth',1.5)
% grid on
% ylabel('Phase (deg)')
% xlabel('Frequency (Hz)')


