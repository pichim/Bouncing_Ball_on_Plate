clc, clear all

% diverse Auswertungsskripts basierend auf serial stream kommunikation.
% Variabelnzuweisungen müssen im cpp / matlab angepasst werden.

%%

port = '/dev/ttyUSB0'; % port = 'COM12';
%port = '/dev/ttyACM0';
baudrate = 2e6;.......


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
legend('Servo 1 PWM', 'Servo 2', 'Servo 1 Rad', 'Location', 'best')

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
xlim([1, 100])
% sgtitle (Super-Title) setzt den Titel mittig über beide Subplots!
title('Bode Diagramm Servo', 'FontWeight', 'bold')


% Figure 6: Bode Diagramm Servo in Hz
figure(6)
clf; % Löscht alten Inhalt, um Skalierungsfehler zu vermeiden

% 1. Bode-Optionen für die Anzeige erstellen
opts = bodeoptions;
opts.FreqUnits = 'Hz';       % Setzt die Anzeige-Einheit auf Hz
opts.XLim = [0.1, 100];      % X-Achsen Bereich direkt in den Optionen setzen
opts.Grid = 'on';           % Grid aktivieren

opts.Title.FontSize = 22;
%opts.Title.FontWeight = 'bold';
opts.XLabel.FontSize = 18;
%opts.XLabel.FontWeight = 'bold';
opts.YLabel.FontSize = 18;
%opts.YLabel.FontWeight = 'bold';
opts.TickLabel.FontSize = 18;
%opts.TickLabel.FontWeight = 'bold';

% 2. Plotten mit bodeplot (erlaubt die Übergabe der Optionen)
h = bodeplot(Gest, opts);


h.Responses.LineWidth = 2.5;





% 3. Phasen-Limits anpassen
% Da bodeplot zwei Subplots hat, müssen wir die Achsen finden
ax = findall(gcf, 'Type', 'axes'); 
% Meist ist ax(1) die Phase (unten) und ax(2) die Magnitude (oben)
if ~isempty(ax)
    ylim(ax(1), [-360, 10]); 
end



title('')
sgtitle('Bode Diagram MKS HBL 669', 'FontSize', 22)


%% Figure 7: Kohärenz in db
figure(7)
bodemag(Cest)
grid on
ylim([-20, 10])
xlim([1, 100])
% Hier reicht ein normales title, da es nur ein Graph ist
title('Kohärenz Bode Diagramm Servo')

%% Figure 8: Kohärenz (Absolut 0 bis 1)
figure(8)
clf;

% Daten extrahieren: squeeze entfernt unnötige Dimensionen, abs zur Sicherheit
freq_hz = Cest.Frequency;
coh_absolute = abs(squeeze(Cest.ResponseData));

% Plotten auf einer logarithmischen X-Achse
semilogx(freq_hz, coh_absolute, 'r', 'LineWidth', 2.0)
grid on

% Achsen beschriften und limitieren
xlabel('Frequency (Hz)')
ylabel('Coherence')
title('Coherence Servo Power HD D12')

ylim([0, 1.1]) % 1.1 damit die Linie bei 1.0 nicht am Rand klebt
xlim([0.1, 100])

% Hilfslinie bei 0.6 einfügen (Qualitätsschwelle)
hold on
% line([0.1 1000], [0.6 0.6], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.2)
% legend('Measured Data', 'Treshhold (0.6)')

% --- Schriftgrößen und Beschriftungen ---
title('Coherence Servo MKS HBL 669', 'FontSize', 16, 'FontWeight', 'normal')
xlabel('Frequency (Hz)', 'FontSize', 14)
ylabel('Coherence', 'FontSize', 14)

% Schriftgröße für die Achsen-Ticks (TickLabel) anpassen
set(gca, 'FontSize', 14)

% Achsenlimits (wichtig: xlim NACH set(gca) setzen, falls MATLAB automatisch skaliert)
ylim([0, 1]) 
xlim([1, 100])




%% Daten laden
data260 = load('chirp_260_lpf/data_00.mat');
data184 = load('chirp_184_lpf/data_00.mat');
data94  = load('chirp_94_lpf/data_00.mat');

% Arrays für die Schleife vorbereiten
datasets = {data260, data184, data94};
labels = {'Chirp 260 LPF', 'Chirp 184 LPF', 'Chirp 94 LPF'};
Gest_all = cell(1,3); % Hier speichern wir die 3 Übertragungsfunktionen
Cest_all = cell(1,3);

% Übertragungsfunktionen berechnen
for i = 1:3
    % WICHTIG: Annahme, dass die Variable in der .mat Datei 'data' heißt.
    % Daher greifen wir mit datasets{i}.data darauf zu.
    d = datasets{i}.data; 
    
    t = d.time;
    Ts = mean(diff(t));          
    fs = 1/Ts;                   
    
    u = d.values(:,3);    % Servo-Chirp Input (0-rad)
    y = d.values(:,10);   % roll (rad)
    
    Nest = round(15 / Ts);
    win = hann(Nest);
    noverlap = round(0.5 * Nest);
    
    % tfestimate berechnen
    [gest, freq] = tfestimate(u, y, win, noverlap, [], fs);
    cest = mscohere(u, y, win, noverla    static constexpr float SERVO_PULSE_MIN = 0.25f;
    static constexpr float SERVO_PULSE_MAX = 0.75f; p, [], fs);
    
    % frd-Objekt erstellen und im Cell-Array speichern
    Gest_all{i} = frd(gest, freq, Ts, 'Units', 'Hz');
    Cest_all{i} = frd(cest, freq, Ts, 'Units', 'Hz');
end

% Figure 8: Gemeinsames Bode Diagramm in Hz
figure(8)
clf; 

% 1. Bode-Optionen erstellen
opts = bodeoptions;
opts.FreqUnits = 'Hz';       % Zeigt die X-Achse in Hertz an
opts.XLim = [1, 120];       % xlim von 10^0 (1) bis 1600 Hz

% 2. Plotten mit den definierten Optionen
bodeplot(Gest_all{1}, 'b', Gest_all{2}, 'r', Gest_all{3}, 'g', opts);
grid on

% 3. Phasen-Limits (Y-Achse) erzwingen
% Sucht alle Achsen in der aktuellen Figur (gcf)
ax = findall(gcf, 'Type', 'axes'); 

% ax(1) ist standardmäßig das untere Diagramm (Phase)
ylim(ax(1), [-360, 10]);     % Setzt das Limit von 10 bis -360 für die Phase

% 4. Beschriftung und Legende
%title('Bode Diagramm Servo - Frequenzvergleich', 'FontWeight', 'bold')
title('')
sgtitle('Bode Diagramm Servo - Frequenzvergleich', 'FontSize', 18, 'FontWeight', 'bold')
legend('Chirp 260 LPF', 'Chirp 184 LPF', 'Chirp 94 LPF', 'Location', 'southwest')

% Plot direkt sichern
% rrinnern('Bode_Vergleich_Fig8', gcf);

% 2. Figure 9: Gemeinsamer Kohärenz-Plot
figure(9)
clf;
hold on % Wichtig: Hält den Plot offen, um alle drei Linien einzuzeichnen

% Farben passend zum Bode-Plot definieren
colors = {'b', 'r', 'g'};
labels = {'Chirp 260 LPF', 'Chirp 184 LPF', 'Chirp 94 LPF'};

% Alle drei Kohärenz-Signale plotten
for i = 1:3
    freq_hz = Cest_all{i}.Frequency;
    coh_absolute = abs(squeeze(Cest_all{i}.ResponseData));
    
    % semilogx zeichnet die logarithmische X-Achse
    semilogx(freq_hz, coh_absolute, 'Color', colors{i}, 'LineWidth', 1.5);
end

grid on
xlabel('Frequenz (Hz)')
ylabel('Kohärenz (Faktor 0 bis 1)')
title('Kohärenz Servo - Frequenzvergleich', 'FontWeight', 'bold')

ylim([0, 1.1]) 
xlim([1, 100]) % Angepasst an den Bode-Plot

% Hilfslinie bei 0.6 einfügen
line([1 1600], [0.6 0.6], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.2)

% Legende (die Reihenfolge entspricht den gezeichneten Linien)
legend(labels{1}, labels{2}, labels{3}, 'Grenzbereich (0.6)', 'Location', 'southwest')

hold off % Schließt das Überlagern ab

%% Frequenzspektrum Analyse (Sichtbarkeits-Update)
colors = {'b', 'r', 'g'};
styles = {'-', '--', ':'}; % Durchgezogen, Gestrichelt, Gepunktet
labels = {'Chirp 260 LPF', 'Chirp 184 LPF', 'Chirp 94 LPF'};

% --- Figure 10: PSD des Inputs u ---
figure(10);
clf; hold on;
for i = 1:3
    d = datasets{i}.data;
    u = d.values(:,3);
    [pxx, f] = pwelch(u, win, noverlap, [], fs);
    
    % Falls sie identisch sind, machen unterschiedliche Styles sie sichtbar:
    semilogx(f, 10*log10(pxx), 'Color', colors{i}, ...
             'LineStyle', styles{i}, 'LineWidth', 2);
end
grid on; grid minor;
xlim([1, 100]);
ylabel('PSD (dB re: Einheit^2/Hz)');
xlabel('Frequenz (Hz)');
title('PSD - Input u (Servo-Chirp)', 'FontWeight', 'bold');
legend(labels, 'Location', 'southwest');
hold off;

% --- Figure 11: PSD des Outputs y (Roll) ---
figure(11);
clf; hold on;
for i = 1:3
    d = datasets{i}.data;
    y = d.values(:,10);
    [pyy, f] = pwelch(y, win, noverlap, [], fs);
    
    semilogx(f, 10*log10(pyy), 'Color', colors{i}, 'LineWidth', 1.5);
end
grid on; grid minor;
xlim([1, 100]);
ylabel('PSD (dB re: rad^2/Hz)');
xlabel('Frequenz (Hz)');
title('PSD - Output y (dB)', 'FontWeight', 'bold');
legend(labels, 'Location', 'southwest');
hold off;

% --- Figure 12: CPSD zwischen u und y ---
figure(12);
clf; hold on;
for i = 1:3
    d = datasets{i}.data;
    u = d.values(:,3);
    y = d.values(:,10);
    [pxy, f] = cpsd(u, y, win, noverlap, [], fs);
    
    % Betrag in dB umwandeln
    semilogx(f, 10*log10(abs(pxy)), 'Color', colors{i}, 'LineWidth', 1.5);
end
grid on; grid minor;
xlim([1, 100]);
ylabel('Magnitude (dB)');
xlabel('Frequenz (Hz)');
title('CPSD - u & y (dB)', 'FontWeight', 'bold');
legend(labels, 'Location', 'southwest');
hold off;


%%
clc, clear all, close all;

% Pfade und Einstellungen
folders = {'chirp_184_lpf', 'chirp_powerhd'};
labels = {'Chirp 184 LPF', 'Chirp PowerHD'};
colors = {'r', 'b'}; % Rot für 184, Blau für PowerHD

% Parameter für Spektralanalyse
Nest_factor = 15; % Fenstergröße in Sekunden

% Daten laden und verarbeiten
Gest_compare = cell(1,2);
Cest_compare = cell(1,2);

for i = 1:2
    filepath = fullfile(folders{i}, 'data_00.mat');
    
    if exist(filepath, 'file')
        fprintf('Lade Daten aus: %s\n', filepath);
        load(filepath); % Lädt die Variable 'data'
        
        % Zeit und Sampling
        t = data.time;
        Ts = mean(diff(t));
        fs = 1/Ts;
        
        % Signale (Servo Chirp Input vs. Roll Output)
        u = data.values(:,3);    % Servo Index 3
        y = data.values(:,10);   % Roll Index 10
        
        % Fenster-Konfiguration
        Nest = round(Nest_factor / Ts);
        win = hann(Nest);
        noverlap = round(0.5 * Nest);
        
        % Übertragungsfunktion und Kohärenz schätzen
        [gest, freq] = tfestimate(u, y, win, noverlap, [], fs);
        [cest, ~] = mscohere(u, y, win, noverlap, [], fs);
        
        % In Frequenzgang-Objekte (FRD) umwandeln
        Gest_compare{i} = frd(gest, freq, Ts, 'Units', 'Hz');
        Cest_compare{i} = frd(cest, freq, Ts, 'Units', 'Hz');
    else
        warning('Datei nicht gefunden: %s', filepath);
    end
end

% Figure 100: Gemeinsamer Bode-Plot
figure(100)
clf;
opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.XLim = [0.1, 80]; % Fokus auf relevanten Frequenzbereich
opts.Grid = 'on';

% Plotten
h = bodeplot(Gest_compare{1}, colors{1}, Gest_compare{2}, colors{2}, opts);

% Achsen-Anpassung (Phase auf -360 bis 10 Grad fixieren)
ax = findall(gcf, 'Type', 'axes');
if length(ax) >= 2
    % ax(1) ist meist Phase (unten), ax(2) Magnitude (oben)
    ylim(ax(1), [-360, 45]); 
end

title('')
sgtitle('Vergleich Frequenzgang: 184 LPF vs. PowerHD', 'FontSize', 14, 'FontWeight', 'bold');
legend(labels, 'Location', 'southwest');

% Figure 101: Gemeinsame Kohärenz
figure(101)
clf; hold on;
for i = 1:2
    if ~isempty(Cest_compare{i})
        f_hz = Cest_compare{i}.Frequency;
        coh = abs(squeeze(Cest_compare{i}.ResponseData));
        semilogx(f_hz, coh, 'Color', colors{i}, 'LineWidth', 2);
    end
end
grid on; grid minor;
line([0.1 100], [0.6 0.6], 'Color', [0.5 0.5 0.5], 'LineStyle', '--'); % Güteschwelle
xlabel('Frequenz (Hz)');
ylabel('Kohärenz');
title('Vergleich Kohärenz (Datenqualität)', 'FontWeight', 'bold');
xlim([0.1, 100]);
ylim([0, 1.1]);
legend([labels, 'Grenzwert (0.6)'], 'Location', 'southwest');
hold off;


%%
% 1. Signale definieren (Indizes anpassen falls 'time' mit in values ist)
t          = data.time; 
z_est_now  = data.values(:, 1); % Ball Z [mm]
h_plate    = data.values(:, 3); % Platte Z [mm]
v_plate    = data.values(:, 4); % Platte v [mm/s]

% 2. Fenster vorbereiten
figure('Name', 'Bounce Timing Analysis', 'Color', 'white');

% 3. Linke Y-Achse (Positionen in mm)
yyaxis left
plot(t, z_est_now, 'b', 'LineWidth', 1.5); hold on;
plot(t, h_plate, 'r', 'LineWidth', 2);
ylabel('Höhe [mm]');
ylim([-20 500]); % Achse fixieren, damit die Platte unten gut sichtbar ist

% 4. Rechte Y-Achse (Geschwindigkeit in mm/s)
yyaxis right
plot(t, v_plate, 'g', 'LineWidth', 1.5);
ylabel('Geschwindigkeit Platte [mm/s]');
ylim([-100 2000]); % Skala anpassen, Vmax ist ja meist bei 1000-1500

% 5. Formatierung
title('Timing-Check: Wann trifft der Ball die schlagende Platte?');
xlabel('Zeit [s]');
legend('Ball Z (Links)', 'Platte Z (Links)', 'Platte v (Rechts)', 'Location', 'northwest');
grid on;
hold off;

%% camera test
t = data.time;          % Zeit-Array laden
y = data.values(:, 2);  % actual pos (ist)

% Standardabweichung (das durchschnittliche Rauschen) berechnen
rauschen_std = std(y);
fprintf('Durchschnittliches Rauschen (1 Sigma): %.5f\n', rauschen_std);

R_gemessen = var(y);

% Signal zentrieren (Mittelwert abziehen)
y_centered = y - mean(y);
fprintf('Varianz (1 Sigma): %.5f\n', R_gemessen);

figure(202); clf;

% --- 1. Achse (unten) für die Zeit ---
ax1 = axes;
plot(ax1, t, y_centered, 'b'); 
grid on;
xlabel(ax1, 'Zeit (s)');
ylabel(ax1, 'Abweichung');
title(ax1, ['Kamera Rauschen (Std: ', num2str(rauschen_std), ')']);

% WICHTIG: X-Limit exakt auf Anfang und Ende der Zeit setzen
ax1.XLim = [t(1), t(end)]; 


figure(203); clf;

plot(t, y, 'b', 'LineWidth', 2); % dickere Linie
grid on;

xlabel('Zeit (s)', 'FontSize', 14);
ylabel('Absolute Position (mm)', 'FontSize', 14);
title(sprintf('Originales Kamera-Signal (Mittelwert: %.2f)', mean(y)), ...
      'FontSize', 16);

set(gca, 'FontSize', 13); % Achsen-Zahlen größer
save_all_plots('kamera_rauschen')

%% jitter
% Daten extrahieren
t = data.time;                         % Zeit-Array (s)
proc_time_vision = data.values(:, 3);  % Processing time (vermutlich in ms)
fps_vision       = data.values(:, 4);  % FPS vision (Hz)
update_interval  = data.values(:, 6);  % Sensor Update Interval in C++ (Mikrosekunden)

% 2. Zeitbereich für den Sprung auswählen (Anpassen!)
% Schau in Figure 2 nach, wann ein sauberer Sprung passiert
t_start = 40; % Beispiel: Sprung bei 5.2 Sekunden
t_end   = 43; % Ende der Beobachtung

% Index-Maske erstellen
idx = (t >= t_start) & (t <= t_end);

% Daten ausschneiden und auf t=0 normieren
% Daten ausschneiden
t = t(idx); 
proc_time_vision = proc_time_vision(idx);
fps_vision = fps_vision(idx);
update_interval = update_interval(idx);
% Zeit auf 0 normieren
t_step = t - t(1);

% (Optional) Mikrosekunden in Millisekunden umrechnen für leichtere Lesbarkeit
update_interval_ms = update_interval / 1000.0; 

% Neues Fenster öffnen
figure(300); clf;

% --- Plot 1: Processing Time Vision ---
subplot(3, 1, 1);
plot(t, proc_time_vision, 'b', 'LineWidth', 1.2);
grid on;
ylabel('Zeit (ms)');
title('Vision Processing Time (Python)');
legend('Processing Time', 'Location', 'best');
xlim([t(1) t(end)]);

% --- Plot 2: Kamera FPS ---
subplot(3, 1, 2);
plot(t, fps_vision, 'g', 'LineWidth', 1.2);
grid on;
ylabel('Frequenz (Hz)');
title('Kamera Framerate (Python)');
legend('FPS', 'Location', 'best');
xlim([t(1) t(end)]);

% --- Plot 3: Sensor Update Interval (C++ Jitter) ---
subplot(3, 1, 3);
plot(t, update_interval_ms, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Zeit (s)');
ylabel('Intervall (ms)'); 
title('Sensor Update Intervall im C++ Regler (Jitter)');
legend('Update Intervall', 'Location', 'best');
xlim([t(1) t(end)]);

% Haupttitel für die gesamte Figure
sgtitle('Analyse: Timing und Jitter', 'FontSize', 14, 'FontWeight', 'bold');


% einzelne plots

% --- Figure 301: Processing Time Vision ---
figure(301); clf;
plot(t, proc_time_vision, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Zeit (s)');
ylabel('Zeit (ms)');
title('Vision Processing Time (Python)');
legend('Processing Time', 'Location', 'best');
xlim([t(1) t(end)]);
set(gca, 'FontSize', 12); % Etwas größere Schrift für bessere Lesbarkeit

% --- Figure 302: Kamera FPS ---
figure(302); clf;
plot(t, fps_vision, 'g', 'LineWidth', 1.5);
grid on;
xlabel('Zeit (s)');
ylabel('Frequenz (Hz)');
title('Kamera Framerate (Python)');
legend('FPS', 'Location', 'best');
xlim([t(1) t(end)]);
set(gca, 'FontSize', 12);

% --- Figure 303: Sensor Update Interval (C++ Jitter) ---
figure(303); clf;
plot(t, update_interval_ms, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Zeit (s)');
ylabel('Intervall (ms)'); 
title('Sensor Update Intervall im C++ Regler (Jitter)');
legend('Update Intervall', 'Location', 'best');
xlim([t(1) t(end)]);
set(gca, 'FontSize', 12);

% --- Figure 304: Kamera FPS als Intervall ---
figure(304); clf;
plot(t, 1000 ./ fps_vision, 'g', 'LineWidth', 1.5); % <-- Hier ist der Punkt wichtig!
grid on;
xlabel('Zeit (s)');
ylabel('Intervall (ms)');
title('Kamera Framerate als Intervall (Python)');
legend('Intervall', 'Location', 'best');
xlim([t(1) t(end)]);
set(gca, 'FontSize', 12);

%
% --- Figure 305: Vergleich Kamera-Intervall vs. C++ Jitter ---
figure(305); clf;
hold on; % WICHTIG: Erlaubt das Zeichnen mehrerer Graphen in einem Plot

% Beide Linien zeichnen
plot(t, 1000 ./ fps_vision, 'g', 'LineWidth', 1.5);
plot(t, update_interval_ms, 'r', 'LineWidth', 1.5);

grid on;
xlabel('Zeit (s)');
ylabel('Intervall (ms)');
title('Vergleich: Kamera Intervall (Python) vs. Sensor Update (C++)');

% Legende hinzufügen (Reihenfolge entspricht den plot-Aufrufen)
legend('Kamera Intervall (Python)', 'Update Intervall (C++)', 'Location', 'best');

% X-Achse exakt auf die Messdauer begrenzen
xlim([t(1) t(end)]);
set(gca, 'FontSize', 12);

hold off; % Hold on wieder ausschalten für zukünftige Plots

