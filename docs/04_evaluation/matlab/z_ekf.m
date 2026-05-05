%% Physik-Check: Kalman Filter OHNE Delay-Kompensation
% -------------------------------------------------------------------------
% 1. Parameter
% -------------------------------------------------------------------------
g = 9810;             
Ts = 0.001;           
k_drag = 0.064 / 1000; 

% Filter Tuning
% Da R_var bei dir EXTREM klein ist (1e-6), wird der Filter 
% fast wie ein direkter Sensor-Read agieren und Sprünge machen.
R_var = 0.001;        
Q_var = diag([0.01, 60]); 

% Matrizen für den Gain H (nur für das Tuning-Verhältnis)
A_lin = [1, Ts; 0, 1];
C_lin = [1, 0];
H = dlqr(A_lin.', C_lin.', Q_var, R_var).'; 

% -------------------------------------------------------------------------
% 2. Daten & Initialisierung
% -------------------------------------------------------------------------
t        = data.time; 
z_ball   = data.values(:, 3);

x_hat_z   = [z_ball(1); 0]; 
pos_z_est = zeros(size(t));
vel_z_est = zeros(size(t));

% -------------------------------------------------------------------------
% 3. Filter-Schleife (Pure Physik & Update)
% -------------------------------------------------------------------------
for k = 1:length(t)
    
    % --- A. PRÄDIKTION (Das physikalische Modell) ---
    v_alt = x_hat_z(2);
    
    % Drag & Gravitation
    a_drag = -k_drag * v_alt * abs(v_alt); 
    a_total = -g + a_drag; 
    
    % Integration (Vorwärtsschritt)
    z_pred = x_hat_z(1) + v_alt * Ts + 0.5 * a_total * Ts^2;
    v_pred = v_alt + a_total * Ts;
    
    x_hat_z = [z_pred; v_pred];
    
    % --- B. KORREKTUR (Direktes Update alle 20ms) ---
    % Hinweis: Wir nutzen hier KEIN Delay mehr. 
    % Wir vergleichen den JETZIGEN Schätzwert mit dem JETZIGEN Messwert.
    if mod(k, 20) == 0
        y_meas = z_ball(k);
        
        y_err = y_meas - x_hat_z(1); % Messfehler
        x_hat_z = x_hat_z + H * y_err; % Kalman-Korrektur
    end
    
    % Ergebnisse speichern
    pos_z_est(k) = x_hat_z(1);
    vel_z_est(k) = x_hat_z(2);
end

% -------------------------------------------------------------------------
% 4. Plotting
% -------------------------------------------------------------------------
figure(115)
subplot(2,1,1);
plot(t, z_ball, 'Color', [0.7 0.7 0.7], 'Marker', '.', 'LineStyle', 'none');
hold on;
plot(t, pos_z_est, 'b', 'LineWidth', 1.5);
title('Physik-Check: Position (Ohne Delay-Kompensation)');
ylabel('Höhe [mm]');
grid on;

subplot(2,1,2);
plot(t, vel_z_est, 'r', 'LineWidth', 1.5);
title('Geschwindigkeit (v\_est)');
xlabel('Zeit [s]');
ylabel('v [mm/s]');
grid on;

figure(116)
plot(t, z_ball);
title('Bouncing höhe');
xlabel('Zeit [s]');
ylabel('z [mm]');
grid on;
xlim([2 8])



%% Simulation Pingpongball im freien Fall mit Luftwiderstand
% --- 1. Physikalische Parameter für TT-Ball ---
m = 0.0027;         % Masse in kg (2,7 g)
d = 0.04;           % Durchmesser in m (40 mm)
A = pi * (d/2)^2;   % Querschnittsfläche in m^2
cw = 0.47;          % Strömungswiderstandskoeffizient (Standard für Kugel)
rho = 1.225;        % Luftdichte in kg/m^3 (auf Meereshöhe)
g = 9.81;           % Erdbeschleunigung m/s^2

% --- 2. Simulations-Setup ---
dt = 0.001;         % Zeitschritt 1 ms (wie in deinem Kalman-Filter)
t_end = 1.5;        % Simulationsdauer in Sekunden (reicht für TT-Ball völlig)
t = 0:dt:t_end;     % Zeitvektor
N = length(t);

% --- 3. Speicher für Ergebnisse ---
z = zeros(1, N);    % Gefallene Distanz in m (positiv = nach unten)
v = zeros(1, N);    % Fallgeschwindigkeit in m/s

% Berechnung der theoretischen stationären Endgeschwindigkeit
% Tritt ein, wenn Gewichtskraft (F_g) = Luftwiderstand (F_drag)
v_term = sqrt((2 * m * g) / (rho * cw * A));
fprintf('Theoretische Endgeschwindigkeit: %.2f m/s (%.0f mm/s)\n', v_term, v_term * 1000);

% --- 4. Integrations-Schleife (Expliziter Euler) ---
for i = 1:(N-1)
    v_akt = v(i);
    
    % Beschleunigung durch Drag: a = F_drag / m
    % F_drag = 0.5 * rho * cw * A * v^2
    a_drag = (0.5 * rho * cw * A * v_akt^2) / m;
    
    % Resultierende Beschleunigung (Schwerkraft zieht, Drag bremst)
    a_ges = g - a_drag; 
    
    % Update für den nächsten Zeitschritt
    v(i+1) = v_akt + a_ges * dt;
    z(i+1) = z(i) + v_akt * dt + 0.5 * a_ges * dt^2;
end

% --- 5. Umrechnung in Millimeter für deinen Roboter ---
z_mm = z * 1000;
v_mm = v * 1000;
v_term_mm = v_term * 1000;

% --- 6. Plotting ---
figure('Name', 'Freier Fall Pingpongball', 'Color', 'w');

% Plot 1: Distanz
subplot(2,1,1);
plot(t, z_mm, 'b', 'LineWidth', 2);
title('Gefallene Distanz');
ylabel('Distanz [mm]');
grid on;

% Plot 2: Geschwindigkeit
subplot(2,1,2);
plot(t, v_mm, 'r', 'LineWidth', 2);
hold on;
% Horizontale Linie für die stationäre Geschwindigkeit einzeichnen
yline(v_term_mm, 'k--', sprintf('Endgeschwindigkeit: %.0f mm/s', v_term_mm), ...
      'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
title('Fallgeschwindigkeit');
xlabel('Zeit [s]');
ylabel('Geschwindigkeit [mm/s]');
grid on;