clc, clear variables
%% parameters and models

g = 9810; % Erdbeschleunigung in mm/s^2

% model plant: angle -> ball position
% hollow sphere
G_Ball = tf(3*g, [5 0 0]);

% servo data
%load chirp_184_lpf/data_00.mat
load chirp_powerhd/data_00.mat

%% lead-controller design

% --- 1. Gegebene Parameter ---
phi_m = 55;         % Phaxsenreserve in Grad
w_d = 1;          % Durchtrittsfrequenz in rad/s

% --- 2. alpha berechnen ---
% Die Funktion sind() erwartet den Winkel praktischerweise direkt in Grad
alpha = (1 - sind(phi_m)) / (1 + sind(phi_m));

% --- 3. Frequenzen berechnen ---
%w_m = w_d / sqrt(alpha); % Kompensationsverfahren
w_m = w_d;

% Nullstelle (w_z) und Polstelle (w_p)
w_z = w_m * sqrt(alpha);
w_p = w_m / sqrt(alpha);

% --- 4. Zeitkonstanten Tv und Tf berechnen ---
T_v = 1 / w_z;
T_f = 1 / w_p;

% --- 6. Übertragungsfunktionen (Control System Toolbox) ---
s = tf('s');

% Variante 1: G_L in der Pol-Nullstellen-Form
G_L = (s + w_z) / (s + w_p);

% --- 7. Werte zur Kontrolle in der Console ausgeben ---
fprintf('--- Berechnete Werte ---\n');
fprintf('alpha = %.4f\n', alpha);
fprintf('w_m   = %.2f rad/s\n', w_m);
fprintf('w_z   = %.4f rad/s\n', w_z);
fprintf('w_p   = %.4f rad/s\n', w_p);
fprintf('T_v   = %.4f s\n', T_v);
fprintf('T_f   = %.4f s\n', T_f);

% --- 8. Bode-Diagramm ---
figure(1);
bode(G_L);
grid on;
title('Bode-Diagramm des Lead-Reglers');

%% --- Load and Identify Servo ---
%load('chirp_184_lpf/data_00.mat'); 

t = data.time;
Ts = mean(diff(t));          
fs = 1/Ts;                   

% Signale extrahieren
u = data.values(:,3);    % Input
y = data.values(:,10);   % Output

% 1. FRD (Messdaten) berechnen wie bisher
Nest = round(15 / Ts);
win = hann(Nest);
noverlap = round(0.5 * Nest);
[gest, freq] = tfestimate(u, y, win, noverlap, [], fs);
Gest = frd(gest, freq, Ts, 'Units', 'Hz');

% 2. Parametrisches Modell (G_servo) schätzen
% WICHTIG: u und y statt u_servo/y_servo nutzen!
z = iddata(y - mean(y), u - mean(u), Ts);

% --- Manueller Beschnitt der Daten ---
f_min = 1; % Hz
f_max = 50; % Hz (Etwas Puffer um deine 1-1.5 Hz)

% Indizes finden, die in unserem Wunschbereich liegen
idx = (Gest.Frequency >= f_min) & (Gest.Frequency <= f_max);

% Ein neues, kleineres FRD-Objekt erstellen
Gest_focus = frd(Gest.ResponseData(idx), Gest.Frequency(idx), Ts, 'Units', 'Hz');

np = 2;
nz = 0;
% Jetzt ohne WeightingFilter schätzen (da die Daten eh schon beschnitten sind)
G_Servo = tfest(Gest_focus, np, nz, 'IODelay', NaN);

fprintf('Servo Model Generated. Fit: %.2f%%\n', G_Servo.Report.Fit.FitPercent);

% --- Korrigierter Plot-Teil ---
figure(2)
clf; % Fenster leeren

% 1. Grafik-Optionen erstellen (Wichtig: bodeoptions, nicht tfestOptions!)
plotOpts = bodeoptions;
plotOpts.FreqUnits = 'Hz';
plotOpts.Grid = 'on';
plotOpts.XLim = [0.1, 100];

% --- SCHRIFTGRÖSSEN DIREKT HIER SETZEN ---
plotOpts.Title.FontSize = 16;
%plotOpts.Title.FontWeight = 'bold';
plotOpts.XLabel.FontSize = 14;
%plotOpts.XLabel.FontWeight = 'bold';
plotOpts.YLabel.FontSize = 14;
%plotOpts.YLabel.FontWeight = 'bold';
plotOpts.TickLabel.FontSize = 14;
%plotOpts.TickLabel.FontWeight = 'bold';

% 2. Der Vergleichs-Plot
% Wir nutzen bodeplot mit den richtigen Optionen
% h = bodeplot(Gest, 'b', G_Servo, 'r--', plotOpts);
h = bodeplot(Gest, 'b', G_Servo, 'r--', plotOpts);

% --- LINIENDICKE ANPASSEN ---
% Wir suchen alle Linien-Objekte, die im Bode-Plot gezeichnet wurden
lineHandles = findobj(h, 'Type', 'line');
set(lineHandles, 'LineWidth', 2.0); % <--- Hier deine Wunschdicke eintragen (z.B. 2.0 oder 2.5)

% 3. Phasen-Limit manuell erzwingen (da bodeoptions das manchmal ignoriert)
ax = findall(gcf, 'Type', 'axes');
if ~isempty(ax)
    % ax(1) ist meistens die Phase
    ylim(ax(1), [-360, 10]);
end
title('')
sgtitle('Bode Diagram Servo MKS HBL 669', 'FontSize', 16);
legend('Measured Data (G_meas)', 'Model (G_servo)', 'Location', 'southwest');



%%
Kp = 0.04;
fcut = 4; % Cutoff-Frequenz in Hz
tau = 1 / (2 * pi * fcut)

% Roll-off Frequenz
tau_ro = 1 / (2 * pi * 1 * fcut) 

% Der Roll-Off Filter (PT1-Glied), der hohe Frequenzen dämpft
F_RollOff = 1 / (tau_ro * s + 1);

% Zum Vergleich auch das reine DT1 Glied
G_DT1 = s / (tau * s + 1);

% Ki = 0.04;
% z = 5;
% Gi = Ki * tf([1 z],[1 0]);


% --- Kombination (Lead-Regler + RO-Filter) ---

G_C = Kp * G_L * F_RollOff;

% Vorfilter für Nullstellenkompensation berechnen:

s = tf('s');
z = zero(G_C);
G_V_dyn = 1 / (s - z(1) * 1);

% für später
% G_V_dyn = 1 / ((s - z(1)) * (s - z(2)));
k_vorfilter = 1 / dcgain(G_V_dyn);

G_V = k_vorfilter * G_V_dyn;

% --- Visualisierung im Bode-Diagramm ---
figure(3);
% Variablen hier korrigiert!
bode(G_L, 'b', G_C, 'r--', F_RollOff, 'k:');
grid on;
title('Bode-Diagramm: Regler-Bausteine');
legend('Reiner Lead (G_L)', 'Kompletter Regler (G_C)', 'Nur RollOff (G_RollOff)');

% Kamera Totzeit
L_kamera = 0.016;
G_delay_kamera = tf(1, 1, 'IODelay', L_kamera);

% Umrechnung Grad zu Rad (cpp RECHNET MIT grad/mm)
Umrechnung_Grad_zu_Rad = pi / 180;

% Plant
G_Plant = G_Ball * G_Servo * Umrechnung_Grad_zu_Rad; % * G_delay_kamera




% --- Analyse des Offenen Regelkreises ---
figure(4); clf;
G_OpenLoop = G_C * G_Plant;
margin(G_OpenLoop);
grid on;

% Hole die Achsen (Subplots) des aktuellen Figures
ax = findall(gcf, 'Type', 'axes'); 

% WICHTIG: 
% ax(1) ist der Phasen-Plot (unten)
% ax(2) ist der Amplituden-Plot (oben)

% 1. X-Achse (Frequenz) für BEIDE Plots setzen
set(ax, 'XLim', [0.1, 100]); 

% 2. Y-Achse für die Phase (unten) setzen
ylim(ax(1), [-180, -90]); 

% 3. Y-Achse für die Amplitude (oben) setzen
% (Hier kannst du deine gewünschten dB-Grenzen eintragen)
ylim(ax(2), [-60, 40]); 

title(ax(2), 'Offener Regelkreis (Strecke + Regler)'); % Titel gehört auf den oberen Plot


%% Hole die Verstärkung (Magnitude) bei der Wunschfrequenz w_d
[mag, ~] = bode(G_OpenLoop, w_d);

% Berechne das korrekte Kp (Kehrwert der Verstärkung)
Kp = 1 / mag;

fprintf('Berechnetes Kp für wd = %.2f: %.4f\n', w_d, Kp);



%% Setpoint filter of ball position

Gcl = feedback(G_OpenLoop, 1);

Tf = 1.0;
F = tf(1, [Tf, 1]); % 1 / (Tf*s + 1)

figure(5)

step(Gcl, G_V * Gcl), grid on, legend('Gcl', 'F * Gcl')


%% --- Closed-Loop Bode-Diagramm ---
% 1. Geschlossene Systeme definieren
Gcl = feedback(G_OpenLoop, 1);
Gcl_mit_Vorfilter = G_V * Gcl;

% 2. Figure erstellen
figure(6);
clf;

% 3. Grafik-Optionen für saubere Achsen definieren
plotOpts = bodeoptions;
plotOpts.FreqUnits = 'Hz'; % Frequenzachse in Hz statt rad/s
plotOpts.Grid = 'on';
plotOpts.XLim = [0.1, 100]; % Gleicher Fokus wie bei deinen restlichen Plots

% Schriftgrößen anpassen (analog zu deinem Servo-Plot)
plotOpts.Title.FontSize = 16;
plotOpts.Title.FontWeight = 'bold';
plotOpts.XLabel.FontSize = 16;
plotOpts.YLabel.FontSize = 16;
plotOpts.TickLabel.FontSize = 14;

% 4. Bode-Plot erzeugen
bodeplot(Gcl, 'b-', Gcl_mit_Vorfilter, 'r-', plotOpts);

hLines = findobj(gcf, 'Type', 'line');
set(hLines, 'LineWidth', 2);

% 5. Titel und Legende hinzufügen
title('');
sgtitle('Closed-Loop Tracking Performance Bode Diagram', 'FontSize', 16, 'FontWeight', 'bold');
legend('Without Prefilter  (G_{cl})', 'With Prefilter (F_{sp} * G_{cl})', 'Location', 'southwest', 'FontSize', 12);

%% --- Closed-Loop Pole-Zero Map (mit Pade-Approximation für Totzeit) ---
% 1. Totzeiten approximieren, da pzplot keine echten Delays plotten kann
Ordnung_Pade = 2; % 2. Ordnung reicht für die Pol-Analyse meist völlig aus
G_Plant_approx = pade(G_Plant, Ordnung_Pade);

% 2. Geschlossene Kreise mit der approximierten Strecke berechnen
G_OpenLoop_approx = G_C * G_Plant_approx;
Gcl_approx = feedback(G_OpenLoop_approx, 1);
Gcl_mit_Vorfilter_approx = G_V * Gcl_approx;

% 3. Figure erstellen
figure(7);
clf;

% 4. Beide Systeme an pzplot übergeben (Reihenfolge getauscht, damit Blau oben liegt!)
h_pz = pzplot(Gcl_mit_Vorfilter_approx, 'ro', Gcl_approx, 'bx'); 
grid on;

% 5. Text-Formatierung (Labels und Titel)
title('Pole-Zero Map of the Closed-Loop (Pade-Approx)', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis (\sigma)', 'FontSize', 16);
ylabel('Imaginary Axis (j\omega)', 'FontSize', 16);

% Legende hinzufügen
legend('Mit Vorfilter (G_V * G_{cl})', 'Ohne Vorfilter (G_{cl})', 'Location', 'best', 'FontSize', 14);

% --- ERST GANZ AM ENDE: ALLES RADIKAL VERGRÖSSERN ---

% 1. Marker (Kreuze und Kreise) massiv vergrößern
all_lines = findall(gcf, 'Type', 'Line');
set(all_lines, 'LineWidth', 2.5, 'MarkerSize', 10); % MarkerSize 12 ist deutlich sichtbar!

% 2. Die Achsenzahlen (Ticks) unübersehbar groß machen
real_ax = findobj(gcf, 'Type', 'axes');
if ~isempty(real_ax)
    set(real_ax, 'FontSize', 18); % Zahlen an den Achsen auf 18
end

%% Führungsverhalten / Störverhalten 

% 1. Führungsverhalten (Setpoint Tracking)
G_CL_tracking = feedback(G_C * G_Plant, 1);

% 2. Störverhalten (Input Disturbance Rejection)
% Störung greift zwischen Regler und Strecke an
G_CL_disturbance = feedback(G_Plant, G_C); 

% Plotting
figure;
subplot(2,1,1);
step(G_CL_tracking);
title('Führungsverhalten: Sprungantwort (Reference Step)');
grid on;

subplot(2,1,2);
step(G_CL_disturbance);
title('Störverhalten: Antwort auf konstante Eingangsstörung (Disturbance Step)');
grid on;

% Statische Abweichungen berechnen (DC Gain)
% Führungsfehler (sollte 0 sein): 1 - dcgain(G_CL_tracking)
% Störfehler (sollte 1/Kp sein): dcgain(G_CL_disturbance)

%% nur Störverhalten
% 1. Berechnung & Simulation
G_CL_disturbance = feedback(G_Plant, G_C); 
[y_dist, t_dist] = step(G_CL_disturbance);

% 2. Plot
figure;
plot(t_dist, y_dist, 'LineWidth', 2);
title('Closed-Loop Disturbance Response', 'FontSize', 16); % Titel richtig eingefügt
grid on;

% 3. Text Formatting
xlabel('Time [s]', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16); 
lgd = legend('Disturbance Response', 'Location', 'northeast');
set(lgd, 'FontSize', 14);
set(gca, 'FontSize', 14); % Schriftgröße der Zahlen an den Achsen

%% step
% 1. Systeme definieren
Gcl = feedback(G_OpenLoop, 1);

% 2. Nur berechnen, nicht direkt plotten!
[y_gcl, t_gcl] = step(Gcl);
[y_F_gcl, t_F_gcl] = step(G_V * Gcl);

% 3. Normal zeichnen
figure(5);
plot(t_gcl, y_gcl, 'LineWidth', 2);
hold on; % Hält den ersten Plot fest, damit der zweite im selben Fenster landet
plot(t_F_gcl, y_F_gcl, 'LineWidth', 2);
hold off;

grid on;
title('Closed-Loop Step Response', 'FontSize', 16);

% 4. Text Formatting (Jetzt funktioniert gca fehlerfrei!)
xlabel('Time [s]', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16); 

lgd = legend('Ohne Vorfilter (G_{cl})', 'Mit Vorfilter (F * G_{cl})', 'Location', 'northeast');
set(lgd, 'FontSize', 14);
set(gca, 'FontSize', 14); % Schriftgröße der Zahlen an den Achsen


%% --- Root Locus Analysis ---
% 1. Totzeiten approximieren, da rlocus keine echten Delays plotten kann
Ordnung_Pade = 2; 
G_Plant_approx = pade(G_Plant, Ordnung_Pade);
G_OL = G_C * G_Plant_approx;

% 2. Figure erstellen
figure(10);
clf;

% 3. Root Locus berechnen und plotten
K_fine = [0, logspace(-4, 1, 2000)]; 

% 3. Root Locus mit dem feinen K-Vektor berechnen und plotten
rlocus(G_OL, K_fine);
grid on;

K_fine = [0, logspace(-4, 1, 20000)]; 

% 3. Root Locus mit dem feinen K-Vektor berechnen und plotten
rlocus(G_OL, K_fine);

% 3. Dämpfungs- und Frequenzraster einblenden
sgrid;


xlim([-3, 0.5]);   % Passt perfekt zu deinen dominanten Polen (bei -1.14 und -0.56)
ylim([-1, 1]);     % Vertikaler Bereich um den Ursprung

% 5. WICHTIG: Geometrisch korrekte Skalierung ERST DANACH erzwingen
axis equal;
axis tight; % Schneidet überschüssige Ränder ab

% 4. Text Formatting & Sizing (Alles auf Englisch)
title('Root Locus of Open-Loop', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis', 'FontSize', 16); % Direkt hier die Wunschgröße für das Label
ylabel('Imaginary Axis', 'FontSize', 16);

% Systemkurven-Linien dicker machen
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 2);

% --- DER TRICK FÜR DIE ZAHLEN ---
% Wir suchen das echte Achsen-Objekt (Axes), das im RLocusPlot versteckt ist
real_ax = findobj(gcf, 'Type', 'axes');
if ~isempty(real_ax)
    set(real_ax, 'FontSize', 14);
end

% Legende formatieren
%lgd = legend('Root Locus', 'Location', 'northeast');
%set(lgd, 'FontSize', 14);



%% --- Closed-Loop Pole-Zero Map (mit Pade-Approximation für Totzeit) ---
% 1. Totzeiten approximieren
Ordnung_Pade = 2; 
G_Plant_approx = pade(G_Plant, Ordnung_Pade);

% 2. Geschlossenen Regelkreis mit Vorfilter berechnen
G_OpenLoop_approx = G_C * G_Plant_approx;
Gcl_approx = feedback(G_OpenLoop_approx, 1);
Gcl_mit_Vorfilter_approx = G_V * Gcl_approx;

% --- POLE UND NULLSTELLEN EXTRAHIEREN (Nur mit Vorfilter) ---
[p_mit, z_mit] = pzmap(Gcl_mit_Vorfilter_approx)

% 3. Figure erstellen
figure(7);
clf;
hold on; 
grid on;

% Hilfslinien für die Achsen (erleichtert die Stabilitätsbeurteilung)
xline(0, '--k', 'LineWidth', 1); 
yline(0, '--k', 'LineWidth', 1);

% 4. System mit Vorfilter in Rot zeichnen
h_p_mit = plot(real(p_mit), imag(p_mit), 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);
h_z_mit = plot(real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2.5, 'MarkerSize', 10);

% 5. Text-Formatierung (Labels und Titel)
title('Pole-Zero Map with Prefilter (F_{SP} * G_{cl})', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis (\sigma)', 'FontSize', 14);
ylabel('Imaginary Axis (j\omega)', 'FontSize', 14);

% Legende für Pole und Nullstellen anpassen
legend([h_p_mit, h_z_mit], 'Poles (\times)', 'Zeros (\circ)', 'Location', 'best', 'FontSize', 14);

% Achsenskalierung symmetrisch anpassen
axis equal; 

% 6. Die Achsenzahlen (Ticks) groß machen
set(gca, 'FontSize', 18);


%% --- Hauptplot (Gesamtübersicht) ---
figure(11);
clf;
hold on; grid on;

% 1. Gesamte Daten plotten (Hauptachse)
h_p_mit = plot(real(p_mit), imag(p_mit), 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);
h_z_mit = plot(real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2.5, 'MarkerSize', 10);

title('Pole-Zero Map with Prefilter', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis (\sigma)', 'FontSize', 14);
ylabel('Imaginary Axis (j\omega)', 'FontSize', 14);
set(gca, 'FontSize', 14);
axis equal;
ylim([-65, 65])
%% --- NEU: BILD-IN-BILD (ZOOM AUF DEN URSPRUNG) ---
% Position: [links, unten, breite, höhe] als Anteil des Fensters (0 bis 1)
% Platziert das Inset oben rechts, falls dort Platz frei ist:
ax_inset = axes('Position', [0.6, 0.34, 0.28, 0.32]); 
box on; % Rahmen um das kleine Bild zeichnen
hold on; grid on;

% Dieselben Daten im Inset nochmals plotten
plot(ax_inset, real(p_mit), imag(p_mit), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
plot(ax_inset, real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2, 'MarkerSize', 10);

% !!! WICHTIG: Grenzen des Insets extrem eng um den Nullpunkt ziehen !!!
xlim(ax_inset, [-1.5, 0]);  % Werte an deine dominanten Pole anpassen!
ylim(ax_inset, [-1, 1]);  % Werte an deine dominanten Pole anpassen!

% Beschriftung des Insets (kleiner halten)
title(ax_inset, 'Zoom near Origin', 'FontSize', 14);
set(ax_inset, 'FontSize', 14);


%% --- Closed-Loop Analyse (Step & Bode nebeneinander) ---
% 1. Systeme definieren
Gcl = feedback(G_OpenLoop, 1);
Gcl_mit_Vorfilter = G_V * Gcl;

% 2. Figure erstellen und direkt breiter machen (für 2 Plots nebeneinander)
figure('Name', 'Closed-Loop Analysis', 'Position', [100, 100, 1200, 500]);
clf;

% ==========================================.1H
% SUBPLOT 1: Step Response (Links)
% ==========================================
subplot(1, 2, 1);

% Berechnen
[y_gcl, t_gcl] = step(Gcl);
[y_F_gcl, t_F_gcl] = step(Gcl_mit_Vorfilter);

% Zeichnen
plot(t_gcl, y_gcl, 'b-', 'LineWidth', 2);
hold on; 
plot(t_F_gcl, y_F_gcl, 'r-', 'LineWidth', 2);
hold off;
grid on;

% Formatierung
title('Closed-Loop Step Response', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16); 
set(gca, 'FontSize', 14); % Schriftgröße der Achsen

% Legende
lgd1 = legend('Without Prefilter (G_{cl})', 'With Prefilter (F_{sp} * G_{cl})', 'Location', 'southeast');
set(lgd1, 'FontSize', 12);


% ==========================================
% SUBPLOT 2: Bode Diagramm (Rechts)
% ==========================================
subplot(1, 2, 2);

% Grafik-Optionen für Bode
plotOpts = bodeoptions;
plotOpts.FreqUnits = 'Hz'; 
plotOpts.Grid = 'on';
plotOpts.XLim = [0.1, 100]; 
plotOpts.Title.String = 'Closed-Loop Tracking Performance (Bode)'; % Titel überschreiben
plotOpts.Title.FontSize = 16;
plotOpts.Title.FontWeight = 'bold';
plotOpts.XLabel.FontSize = 16;
plotOpts.YLabel.FontSize = 16;
plotOpts.TickLabel.FontSize = 14;

% Bode-Plot zwingend in die aktuelle Achse (gca) zeichnen!
bodeplot(gca, Gcl, 'b-', Gcl_mit_Vorfilter, 'r-', plotOpts);

% Legende (MATLAB platziert sie automatisch passend im Bode-Plot)
lgd2 = legend('Without Prefilter (G_{cl})', 'With Prefilter (F_{sp} * G_{cl})', 'Location', 'southwest');
set(lgd2, 'FontSize', 12);

% ==========================================
% ABSCHLUSS: Liniendicke global für den Bode-Plot anpassen
% ==========================================
% Sucht alle Linien in der gesamten Figure und setzt sie auf 2
hLines = findobj(gcf, 'Type', 'line');
set(hLines, 'LineWidth', 2);

%% --- Custom Pole-Zero Map: Einfluss des Vorfilters ---
% 1. Totzeiten approximieren
Ordnung_Pade = 2; 
G_Plant_approx = pade(G_Plant, Ordnung_Pade);

% 2. Beide geschlossenen Regelkreise berechnen
G_OpenLoop_approx = G_C * G_Plant_approx;
Gcl_approx = feedback(G_OpenLoop_approx, 1);
Gcl_mit_Vorfilter_approx = G_V * Gcl_approx;

% 3. Pole und Nullstellen mathematisch extrahieren
[p_ohne, z_ohne] = pzmap(Gcl_approx);
[p_mit, z_mit] = pzmap(Gcl_mit_Vorfilter_approx);

% 4. Figure erstellen
figure(12);
clf;
hold on; 
grid on;

% Hilfslinien für das Achsenkreuz (Stabilitätsgrenze)
xline(0, '--k', 'LineWidth', 1.5); 
yline(0, '--k', 'LineWidth', 1.5);

% --- PLOTTEN ---
% Zuerst das System MIT Vorfilter (Rot, im Hintergrund)
h_p_mit = plot(real(p_mit), imag(p_mit), 'rx', 'LineWidth', 2.5, 'MarkerSize', 12);
h_z_mit = plot(real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2.5, 'MarkerSize', 12);

% Danach das System OHNE Vorfilter (Blau, im Vordergrund)
h_p_ohne = plot(real(p_ohne), imag(p_ohne), 'bx', 'LineWidth', 2.5, 'MarkerSize', 8);
h_z_ohne = plot(real(z_ohne), imag(z_ohne), 'bo', 'LineWidth', 2.5, 'MarkerSize', 8);

hold off;

% 5. Text-Formatierung & Sizing (Analog zu deinen restlichen Plots)
title('Influence of Prefilter on Closed-Loop Poles and Zeros', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis (\sigma)', 'FontSize', 14);
ylabel('Imaginary Axis (j\omega)', 'FontSize', 14);

% Symmetrische Achsenskalierung und Fokus auf deine komplexen Pole
axis equal;
ylim([-65, 65]); 

% Achsenzahlen (Ticks) unübersehbar groß machen
set(gca, 'FontSize', 18);

% Legende so aufbauen, dass man die Systeme sauber unterscheidet
legend([h_p_ohne, h_p_mit], ...
       'Without Prefilter (G_{cl})', ...
       'With Prefilter (G_V * G_{cl})', ...
       'Location', 'best', 'FontSize', 14);

%% --- Disturbance Rejection Analysis (Step & Bode nebeneinander) ---
% 1. Störübertragungsfunktion definieren 
% (Störung greift zwischen Regler und Strecke an)
G_CL_disturbance = feedback(G_Plant, G_C); 

% 2. Figure erstellen und breiter machen (für 2 Plots nebeneinander)
figure('Name', 'Disturbance Rejection Analysis', 'Position', [150, 150, 1200, 500]);
clf;

% ==========================================
% SUBPLOT 1: Disturbance Step Response (Links)
% ==========================================
subplot(1, 2, 1);
% Berechnen
[y_dist, t_dist] = step(G_CL_disturbance);

% Zeichnen
plot(t_dist, y_dist, 'b-', 'LineWidth', 2);
grid on;

% Formatierung
title('Disturbance Step Response', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16); 
set(gca, 'FontSize', 14); % Schriftgröße der Achsen

% Legende
lgd_dist = legend('Disturbance Response', 'Location', 'northeast');
set(lgd_dist, 'FontSize', 12);

% ==========================================
% SUBPLOT 2: Disturbance Bode Diagramm (Rechts)
% ==========================================
subplot(1, 2, 2);

% Grafik-Optionen für Bode
plotOpts = bodeoptions;
plotOpts.FreqUnits = 'Hz'; 
plotOpts.Grid = 'on';
plotOpts.XLim = [0.1, 100]; 
plotOpts.Title.String = 'Disturbance Rejection Performance (Bode)'; 
plotOpts.Title.FontSize = 16;
plotOpts.Title.FontWeight = 'bold';
plotOpts.XLabel.FontSize = 16;
plotOpts.YLabel.FontSize = 16;
plotOpts.TickLabel.FontSize = 14;

% Bode-Plot zwingend in die aktuelle Achse (gca) zeichnen!
bodeplot(gca, G_CL_disturbance, 'b-', plotOpts);

% ==========================================
% ABSCHLUSS: Liniendicke global für den Bode-Plot anpassen
% ==========================================
% Sucht alle Linien im rechten Subplot und setzt sie auf 2
hLines = findobj(gca, 'Type', 'line');
set(hLines, 'LineWidth', 2);



%% --- Hauptplot (Gesamtübersicht) mit separatem Vorfilter-Pol ---
% 1. Totzeiten approximieren und Kreise berechnen
Ordnung_Pade = 2; 
G_Plant_approx = pade(G_Plant, Ordnung_Pade);
G_OpenLoop_approx = G_C * G_Plant_approx;

Gcl_approx = feedback(G_OpenLoop_approx, 1);
Gcl_mit_Vorfilter_approx = G_V * Gcl_approx;

% 2. Pole und Nullstellen extrahieren
[p_ohne, z_mit] = pzmap(Gcl_approx); % Zeros sind mit/ohne Vorfilter identisch
[p_mit, ~] = pzmap(Gcl_mit_Vorfilter_approx);

% --- TRICK: Den zusätzlichen Vorfilter-Pol isolieren ---
% Wir suchen den Pol in 'p_mit', der am weitesten von allen Polen in 'p_ohne' entfernt ist
[~, idx] = max(min(abs(p_mit - p_ohne.'), [], 2));
p_prefilter = p_mit(idx);       % Das ist der reine Vorfilter-Pol
p_system = p_mit(p_mit ~= p_prefilter); % Das sind die restlichen Systempole

% 3. Figure erstellen
figure(11);
clf;
hold on; grid on;

% 4. Gesamte Daten plotten (Hauptachse)
h_p_sys = plot(real(p_system), imag(p_system), 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);
h_z_mit = plot(real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2.5, 'MarkerSize', 10);
% Der Vorfilter-Pol wird hier in giftgrün ('g') oder magenta ('m') hervorgehoben:
h_p_fsp = plot(real(p_prefilter), imag(p_prefilter), 'bx', 'LineWidth', 3.5, 'MarkerSize', 10);

title('Pole-Zero Map with Prefilter', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Real Axis (\sigma)', 'FontSize', 14);
ylabel('Imaginary Axis (j\omega)', 'FontSize', 14);
set(gca, 'FontSize', 14);
axis equal;
ylim([-65, 65])

% Saubere Legende auf der Hauptachse
legend([h_p_sys, h_z_mit, h_p_fsp], ...
       'System Poles (\times)', 'System Zeros (\circ)', 'Prefilter Pole (\times)', ...
       'Location', 'best', 'FontSize', 12);

% --- BILD-IN-BILD (ZOOM AUF DEN URSPRUNG) ---
ax_inset = axes('Position', [0.6, 0.34, 0.28, 0.32]); 
box on; 
hold on; grid on;

% Dieselben Daten im Inset nochmals plotten
plot(ax_inset, real(p_system), imag(p_system), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
plot(ax_inset, real(z_mit), imag(z_mit), 'ro', 'LineWidth', 2, 'MarkerSize', 10);
plot(ax_inset, real(p_prefilter), imag(p_prefilter), 'bx', 'LineWidth', 3, 'MarkerSize', 10);

% Grenzen extrem eng um die dominanten Pole und den Vorfilter-Pol ziehen
xlim(ax_inset, [-1.5, 0]);  
ylim(ax_inset, [-1, 1]);  
title(ax_inset, 'Zoom near Origin', 'FontSize', 14);
set(ax_inset, 'FontSize', 14);
