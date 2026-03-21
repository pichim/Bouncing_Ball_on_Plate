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
Nest = round(15 / Ts);
win = hann(Nest);
noverlap = round(0.5 * Nest);

[gest, freq] = tfestimate(u, y, win, noverlap, [], 1/Ts);
cest = mscohere(u, y, win, noverlap, [], 1/Ts);

Gest = frd(gest, freq, Ts, 'Units', 'Hz');

Cest = frd(cest, freq, Ts, 'Units', 'Hz');

% Figure 6: Bode Diagramm Servo
figure(6)
bode(Gest)
grid on
ylim([-360, 10]) % Wirkt auf die Phase
xlim([1, 1000])
% sgtitle (Super-Title) setzt den Titel mittig über beide Subplots!
title('Bode Diagramm Servo', 'FontWeight', 'bold')

%% Figure 7: Kohärenz in db
figure(7)
bodemag(Cest)
grid on
ylim([-20, 10])
xlim([1, 1000])
% Hier reicht ein normales title, da es nur ein Graph ist
title('Kohärenz Bode Diagramm Servo')

%% Figure 7: Kohärenz (Absolut 0 bis 1)
figure(7)
clf;

% Daten extrahieren: squeeze entfernt unnötige Dimensionen, abs zur Sicherheit
freq_hz = Cest.Frequency;
coh_absolute = abs(squeeze(Cest.ResponseData));

% Plotten auf einer logarithmischen X-Achse
semilogx(freq_hz, coh_absolute, 'r', 'LineWidth', 2.0)
grid on

% Achsen beschriften und limitieren
xlabel('Frequenz (Hz)')
ylabel('Kohärenz (Faktor 0 bis 1)')
title('Kohärenz Servo (Absolut)', 'FontWeight', 'bold')

ylim([0, 1.1]) % 1.1 damit die Linie bei 1.0 nicht am Rand klebt
xlim([1, 1000])

% Hilfslinie bei 0.6 einfügen (Qualitätsschwelle)
hold on
line([1 1000], [0.6 0.6], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.2)
legend('Messdaten', 'Grenzbereich (0.6)')

%%
save_all_plots('chirp_184hz_lpf_125Hz');

%%
function save_all_plots(prefix)
    % Falls kein Präfix angegeben wurde
    if nargin < 1, prefix = 'Plot'; end
    
    % Erstelle einen Ordner 'Results', falls er nicht existiert
    if ~exist('Results', 'dir')
        mkdir('Results');
    end
    
    % Finde alle offenen Figure-Handles
    figHandles = findobj('Type', 'figure');
    
    for i = 1:length(figHandles)
        fig = figHandles(i);
        figNum = fig.Number;
        
        % --- NEU: Liniendicke für alle Linien in dieser Figure anpassen ---
        % Findet alle Linien (Plots, Hilfslinien, etc.)
        allLines = findobj(fig, 'Type', 'line');
        set(allLines, 'LineWidth', 2.0); % Hier Wert anpassen (Standard ist 0.5)
        
        % Optional: Auch die Schriftgröße der Achsen für bessere Lesbarkeit erhöhen
        allAxes = findobj(fig, 'Type', 'axes');
        set(allAxes, 'FontSize', 12, 'FontWeight', 'bold');
        
        % Dateiname generieren
        filename = sprintf('Results/%s_Fig%d.png', prefix, figNum);
        
        % Speichern mit hoher Qualität
        exportgraphics(fig, filename, 'Resolution', 300);
        
        fprintf('Gespeichert: %s (Liniendicke angepasst)\n', filename);
    end
end