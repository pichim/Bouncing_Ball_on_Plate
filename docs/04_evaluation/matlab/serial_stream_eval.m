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
                'Std. %0.0f mus, ', ...
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

figure(2)
plot(data.time, data.values(:, ind.servo_commands))

figure(3)
%blau
%orange
%gelb

subplot(311)
plot(data.time, data.values(:, ind.gyro))
subplot(312)
plot(data.time, data.values(:, ind.acc))
subplot(313)
plot(data.time, data.values(:, ind.rpy))


%%

addpath mahony/


para.kp = 0.1592 * 2.0 * pi;
para.ki = 0.0;

rpy0 = data.values(1, ind.rpy);
quat0 = rpy2quat(rpy0).';

[quatRP , biasRP] = mahonyRP(data.values(:,ind.gyro ), data.values(:,ind.acc), para, Ts, quat0);

rpyRP  = quat2rpy(quatRP );

figure(4)
plot(data.time, [data.values(:, ind.rpy), rpyRP])


%% Servo Frequency Response (Chirp → Gyro Z)

% --- Signale ---
t = data.time;
Ts = mean(diff(t));          % Samplingzeit
fs = 1/Ts;                   % Samplingfrequenz

fmin = 1;
fmax = 10;



u = (data.values(:,1)) * pi * 110 / 180;   % Servo-Chirp Input (0-1) normiert auf rad
y = detrend(data.values(:,11));   % Gyro Z (rad/s) 6  // Yaw (rad) 12

figure(5)
plot(t,u)



% --- Frequenzgang-Schätzung ---
N = length(u);
window = hann(floor(N/4));
noverlap = floor(length(window)/2);
nfft = length(window);

[H,f] = tfestimate(u, y, window, noverlap, nfft, fs);


idx = (f >= fmin) & (f <= fmax);

% --- Bode Plot ---
figure(6)
subplot(2,1,1)
semilogx(f(idx),20*log10(abs(H(idx))),'LineWidth',1.5)
grid on
ylabel('Magnitude (dB)')
title('Servo Frequency Response')

subplot(2,1,2)
semilogx(f(idx),angle(H(idx))*180/pi,'LineWidth',1.5)
grid on
ylabel('Phase (deg)')
xlabel('Frequency (Hz)')


