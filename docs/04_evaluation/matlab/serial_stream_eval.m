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
xlim([0.1, 100])
% sgtitle (Super-Title) setzt den Titel mittig über beide Subplots!
title('Bode Diagramm Servo', 'FontWeight', 'bold')


% Figure 6: Bode Diagramm Servo in Hz
figure(6)
clf; % Löscht alten Inhalt, um Skalierungsfehler zu vermeiden

% 1. Bode-Optionen für die Anzeige erstellen
opts = bodeoptions;
opts.FreqUnits = 'Hz';       % Setzt die Anzeige-Einheit auf Hz
opts.XLim = [0.1, 10];      % X-Achsen Bereich direkt in den Optionen setzen
opts.Grid = 'on';           % Grid aktivieren

% 2. Plotten mit bodeplot (erlaubt die Übergabe der Optionen)
h = bodeplot(Gest, opts);

% 3. Phasen-Limits anpassen
% Da bodeplot zwei Subplots hat, müssen wir die Achsen finden
ax = findall(gcf, 'Type', 'axes'); 
% Meist ist ax(1) die Phase (unten) und ax(2) die Magnitude (oben)
if ~isempty(ax)
    ylim(ax(1), [-360, 10]); 
end

title('Bode Diagramm Servo (Frequenz in Hz)', 'FontWeight', 'bold')
%% Figure 7: Kohärenz in db
figure(7)
bodemag(Cest)
grid on
ylim([-20, 10])
xlim([1, 1000])
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
xlabel('Frequenz (Hz)')
ylabel('Kohärenz (Faktor 0 bis 1)')
title('Kohärenz Servo (Absolut)', 'FontWeight', 'bold')

ylim([0, 1.1]) % 1.1 damit die Linie bei 1.0 nicht am Rand klebt
xlim([1, 1000])

% Hilfslinie bei 0.6 einfügen (Qualitätsschwelle)
hold on
line([1 1000], [0.6 0.6], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 1.2)
legend('Messdaten', 'Grenzbereich (0.6)')




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
    cest = mscohere(u, y, win, noverlap, [], fs);
    
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


% Speichern mit deiner Funktion
% save_all_plots('CPSD_dB_Final');

%%
save_all_plots('chirp_powerhd');
%%

function save_all_plots(prefix)
    if nargin < 1, prefix = 'Plot'; end
    
    if ~exist('Results', 'dir'), mkdir('Results'); end
    
    figHandles = findobj('Type', 'figure');
    
    for i = 1:length(figHandles)
        fig = figHandles(i);
        figNum = fig.Number;
        
        % 1. Liniendicke (2.0 für gute Sichtbarkeit im Bericht)
        set(findobj(fig, 'Type', 'line'), 'LineWidth', 2.0);
        
        % 2. Alle Achsen-Objekte finden (beim Bode sind das zwei!)
        allAxes = findobj(fig, 'Type', 'axes');
        
        for j = 1:length(allAxes)
            ax = allAxes(j);
            
            % Achsen-Zahlen (Ticks)
            set(ax, 'FontSize', 12, 'FontWeight', 'bold');
            
            % --- TITEL ERZWINGEN ---
            % Wir greifen direkt auf das Title-Objekt der Achse zu
            set(ax.Title, 'FontSize', 16, 'FontWeight', 'bold', 'Visible', 'on');
            
            % Auch X- und Y-Labels vergrößern (wichtig für Bode-Achsen)
            set(ax.XLabel, 'FontSize', 13, 'FontWeight', 'bold');
            set(ax.YLabel, 'FontSize', 13, 'FontWeight', 'bold');
        end
        
        % 3. Super-Title (sgtitle) finden
        % Dieser Titel steht über beiden Bode-Plots
        allSg = findall(fig, 'Tag', 'suptitle'); 
        if ~isempty(allSg)
            set(allSg, 'FontSize', 18, 'FontWeight', 'bold');
        end
        
        % 4. Legenden
        set(findobj(fig, 'Type', 'legend'), 'FontSize', 14, 'FontWeight', 'bold');
        
        % Speichern
        filename = sprintf('Results/%s_Fig%d.png', prefix, figNum);
        exportgraphics(fig, filename, 'Resolution', 300);
        
        fprintf('Gespeichert: %s (Alles vergrößert)\n', filename);
    end
end