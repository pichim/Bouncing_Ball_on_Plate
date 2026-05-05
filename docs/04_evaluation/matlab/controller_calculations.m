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
w_d = 2.4;          % Durchtrittsfrequenz in rad/s

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
f_max = 80.0; % Hz (Etwas Puffer um deine 1-1.5 Hz)

% Indizes finden, die in unserem Wunschbereich liegen
idx = (Gest.Frequency >= f_min) & (Gest.Frequency <= f_max);

% Ein neues, kleineres FRD-Objekt erstellen
Gest_focus = frd(Gest.ResponseData(idx), Gest.Frequency(idx), Ts, 'Units', 'Hz');

np = 2;
nz = 0;
% Jetzt ohne WeightingFilter schätzen (da die Daten eh schon beschnitten sind)
G_Servo = tfest(Gest_focus, np, nz, 'IODelay', NaN);

% G_Servo =
% 
%                           4692 s - 5.057e05
%  exp(-0.008*s) * -----------------------------------
%                  s^3 + 34.35 s^2 + 3188 s - 5.139e05


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
plotOpts.Title.FontSize = 26;
plotOpts.Title.FontWeight = 'bold';
plotOpts.XLabel.FontSize = 20;
plotOpts.XLabel.FontWeight = 'bold';
plotOpts.YLabel.FontSize = 20;
plotOpts.YLabel.FontWeight = 'bold';
plotOpts.TickLabel.FontSize = 20;
plotOpts.TickLabel.FontWeight = 'bold';

% 2. Der Vergleichs-Plot
% Wir nutzen bodeplot mit den richtigen Optionen
h = bodeplot(Gest, 'b', G_Servo, 'r--', plotOpts);

% 3. Phasen-Limit manuell erzwingen (da bodeoptions das manchmal ignoriert)
ax = findall(gcf, 'Type', 'axes');
if ~isempty(ax)
    % ax(1) ist meistens die Phase
    ylim(ax(1), [-360, 10]);
end
title('')
sgtitle(['Servo Model Validation: Fit = ', num2str(G_Servo.Report.Fit.FitPercent, '%.1f'), '%'], 'FontSize', 26);
legend('Measured Data (G_meas)', 'Model (G_servo)', 'Location', 'southwest');



%%
Kp = 0.2383;
fcut = 4; % Cutoff-Frequenz in Hz
tau = 1 / (2 * pi * fcut)

% Roll-off Frequenz
tau_ro = 1 / (2 * pi * 1 * fcut) 

% Der Roll-Off Filter (PT1-Glied), der hohe Frequenzen dämpft
F_RollOff = 1 / (tau_ro * s + 1);

% Zum Vergleich auch das reine DT1 Glied
G_DT1 = s / (tau * s + 1);

% --- Kombination (Lead-Regler + Supervisor Filter) ---
% Die Systeme werden multipliziert (in Reihe geschaltet)
G_C = Kp * G_L * F_RollOff; %

% Vorfilter für Nullstellenkompensation berechnen:

s = tf('s');
z = zero(G_C)
G_V_dyn = 1 / (s - z(1));

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
G_Plant = G_Ball * G_delay_kamera  * G_Servo * Umrechnung_Grad_zu_Rad;




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

step(Gcl, G_V * Gcl, F * Gcl), grid on, legend('Gcl', 'G_V * Gcl', 'F * Gcl')

%% Static (steady-state) kalman filter

% 3*g * u = 5 * ddx
A = [[0 1]; [0 0]];
B = [0; 3/5*g];
C = [1 0];
% D = 0;
Ts = 0.001;

% euler discretization, could also be zoh
A = eye(size(A)) + Ts * A;
B = Ts * B;

% % zoh
% M = expm([A, B; zeros(1,3)] * Ts);
% A = M(1:2,1:2);
% B = M(1:2,3);

% static kalman-filter
R = 0.006 / Ts; % tune here
% Q = B * B.' * 1 * Ts;
Q = diag([1 100]);
H = dlqr(A.', C.', Q, R).';

% extend with disturbance input
Ae = [[A, B]; [0, 0, 1]];
Be = [B; 0];
Ce = [C, 0];

% static kalman-filter for with input disturbance estimator
Qe = Q;
Qe(3,3) = 1e-2; % you need to add the penalty for the disturbance extra!
He = dlqr(Ae.', Ce.', Qe, R).';

% position sensor runs 20-times slower
Ts_pos = 20 * Ts;

G_DT1 = c2d(tf([1 0], [1/(2*pi*1) 1]), Ts, 'tustin');

% - you can also formulate output disturbances
% - you can also formulate the disturbance dynamics not as integrator but
%   as a lowpass type, this sometimes helps with observability

% --- Simulation des Kalman-Filters ---
t_sim = 0:Ts:4; % 2 Sekunden Simulation
true_pos = sin(2*pi*0.5*t_sim); % Die "echte" Bewegung (Sinus)
noise = 0.05 * randn(size(true_pos)); % Kamera-Rauschen simulieren
measured_pos = true_pos + noise;

% Speicher für die Ergebnisse
x_hat = [0; 0]; % Startzustand [Position; Geschwindigkeit]
pos_est = zeros(size(true_pos));
vel_est = zeros(size(true_pos));

% Simulations-Schleife (wie später auf dem Microcontroller)
for k = 1:length(t_sim)
    % 1. Prediction (Physik)
    x_hat = A * x_hat + B * 0; % Hier u=0 angenommen
    
    % 2. Correction (Kamera-Update)
    % Nur alle 20ms ein Update
    if mod(k, 20) == 0
        y_meas = measured_pos(k);
        x_hat = x_hat + H * (y_meas - C * x_hat);
    end
    
    pos_est(k) = x_hat(1);
    vel_est(k) = x_hat(2);
end

figure(10)
subplot(2,1,1)
plot(t_sim, measured_pos, 'g.', t_sim, true_pos, 'k', t_sim, pos_est, 'r', 'LineWidth', 1.5)
title('Kalman-Filter: Positions-Schätzung')
legend('Verrauschte Kamera', 'Echte Position', 'Kalman Schätzung')

subplot(2,1,2)
plot(t_sim, vel_est, 'r', 'LineWidth', 1.5)
title('Vom Kalman-Filter geschätzte Geschwindigkeit')
grid on