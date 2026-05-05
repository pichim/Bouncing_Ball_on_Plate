%% Camera calibration measurements
t = data.time;          
x = data.values(:, 1);  
y = data.values(:, 2);  
z = data.values(:, 5);  

% Referenz (Zentrum) aus den ersten 100 Messungen
x_ref = mean(x(1:100));
y_ref = mean(y(1:100));
z_ref = mean(z(1:100));

% Relative Koordinaten berechnen
x_rel = x - x_ref;
y_rel = y - y_ref;
z_rel = z - z_ref;
distanz = sqrt(x_rel.^2 + y_rel.^2);

% 2. Daten filtern (Bewegung herausrechnen)



% --- Methode B: Manuelle Zeitfenster (Alternativ) ---

 zeitfenster = [0, 20;
                22, 23;
                15, 18;
                21, 23;
                24, 28;
                30, 31
                33, 34;
                37,38;
                40, 42;
                43, 45;
                47, 48;
                49, 50;
                53, 55];
 idx_ruhig = false(size(t));
 for i = 1:size(zeitfenster, 1)
     idx_ruhig = idx_ruhig | (t >= zeitfenster(i,1) & t <= zeitfenster(i,2));
 end

% Wir wenden den Filter auf unsere Daten an
x_clean = x_rel(idx_ruhig);
y_clean = y_rel(idx_ruhig);
z_clean = z_rel(idx_ruhig);
t_clean = t(idx_ruhig);
distanz_clean = distanz(idx_ruhig);

% 3. Plotten - Übersicht (Subplots)
fig_uebersicht = figure('Name', 'Übersicht: Kalibrierung');

subplot(2, 2, 1);
scatter(x_clean, y_clean, 20, 'filled'); % Nur die sauberen Daten
title('Draufsicht (Gefiltert)'); xlabel('X'); ylabel('Y'); axis equal; grid on;

subplot(2, 2, 2);
scatter(t, z, 10, 'filled'); % Scatter ist hier oft besser als Plot
title('Höhe (Z-Achse)'); xlabel('Zeit (s)'); ylabel('Z-Abweichung'); grid on;

subplot(2, 2, [3, 4]);
scatter(t, distanz, 10, 'filled');
title('Distanz zum Zentrum'); xlabel('Zeit (s)'); ylabel('Distanz'); grid on;


% 4. Plotten
% Detail Plot A: Draufsicht
fig_draufsicht = figure('Name', 'Detail: Draufsicht');
scatter(x_clean, y_clean, 40, 'filled', 'MarkerFaceColor', '#0072BD');
title('Draufsicht: Kamerapositionen');
xlabel('X Distanz zum Zentrum'); ylabel('Y Distanz zum Zentrum');
axis equal; grid on;

% Detail Plot B: Höhe
fig_hoehe = figure('Name', 'Detail: Höhe');
scatter(t, z, 20, 'filled', 'MarkerFaceColor', '#D95319');
title('Höhenabweichung des Balls über Zeit');
xlabel('Zeit (s)'); ylabel('Z-Abweichung (Höhe)');
grid on;

% Detail Plot C: Distanz
fig_distanz = figure('Name', 'Detail: Distanz');
scatter(t, distanz, 20, 'filled', 'MarkerFaceColor', '#EDB120');
title('2D-Distanz zum Zentrum');
xlabel('Zeit (s)'); ylabel('Absolute Distanz');
grid on;

%% --- FILTER SCHALTPULT ---
% Hier kannst du jeden Filter einzeln ein- (true) oder ausschalten (false)
nutze_sniper = true;  % Manuelles Löschen
nutze_physik = true;  % Geschwindigkeits-Filter
nutze_hampel = true;   % Statistischer Filter (Hampel)

% 1. Daten einlesen und reinen Z-Fehler berechnen
t = data.time;          
x = data.values(:, 1);  
y = data.values(:, 2);  
z = data.values(:, 5);
x_min = -60;
x_max = 60;
y_min = -60;
y_max = 60;
z_toleranz = 10;


z_ref = mean(z(1:100))
z_fehler = z - z_ref;   
z_bereinigt = z_fehler; % Startwert für unsere Filter-Kette


% 2. Der "Sniper"
idx_sniper = false(size(t)); % Standard: Leere Fehlerliste (Wichtig für den Plot später!)
if nutze_sniper
    idx_sniper = (t >= 36) & (t <= 42);
    z_bereinigt(idx_sniper) = NaN; 
end

% 3. Physik-Filter
idx_speed = false(size(t)); % Standard: Leere Fehlerliste
if nutze_physik
    dt = [1; diff(t)];
    v_z = [0; diff(z_fehler)] ./ dt; 
    max_z_speed = 200; 
    idx_speed = abs(v_z) > max_z_speed;
    z_bereinigt(idx_speed) = NaN;
end

% ZWISCHENSCHRITT: Lücken füllen (Passiert nur, wenn Sniper oder Physik an waren)
if nutze_sniper || nutze_physik
    z_bereinigt = fillmissing(z_bereinigt, 'linear');
end

% 4. Hampel-Filter
idx_hampel = false(size(t)); % Standard: Leere Fehlerliste
if nutze_hampel
    fenster = 500; 
    toleranz = 1.1; 
    [z_gefiltert, idx_hampel] = hampel(z_bereinigt, fenster, toleranz);
else
    % Wenn Hampel AUS ist, reichen wir die Daten einfach unberührt weiter!
    z_gefiltert = z_bereinigt; 
end

% 5. Bounding-Box (X/Y/Z) und Zusammenfassung
% ... [AB HIER BLEIBT ALLES GENAU WIE VORHER] ...

% Zur Sicherheit hier nochmal die Zeile für die Zusammenfassung der Fehler, 
% die jetzt dank der "false"-Standardwerte nie abstürzen wird:
idx_alle_fehler = idx_sniper | idx_speed | idx_hampel;

idx_gueltig = (abs(z_gefiltert) < z_toleranz) & ...
              (x >= x_min) & (x <= x_max) & ...
              (y >= y_min) & (y <= y_max);

% Saubere Daten für das 3D-Mesh extrahieren
x_clean = x(idx_gueltig);
y_clean = y(idx_gueltig);
z_clean = z_gefiltert(idx_gueltig);


%%
%tiefste stelle
x_clean = [ 7.3, 34.2, -40, -62.1, -18.57, 53.66, 59.4, -25.5, 29.5, -18.35, 21, -10];
y_clean = [ -3.97, -51.78, -60.7, 10.3, 49.1, 46, -23.6, 18.9, -23.12, -33.35, 8.21, -65];
z_clean = [ 130.3, 128.3, 127.3, 127.2, 130.2, 128.67, 126.6, 130, 129, 129.8, 130, 129.2];

%höchste stelle
%x_clean = [ 4.8, -17.1, 64.5, 80.5, 18.7, -36.6, -58.7];
%y_clean = [ -4.20, 60, 33.7, 0.5, -70.7, -62, 3.7];
%z_clean = [ 245, 245, 243.7, 242.8, 244.7, 244, 244];

%home pos
x_clean = [6.55, -36.73, -68.08, -16.63, 63.64, 73.61, 26.31, -17.38, -29.74, -29.85, -31.23, -5.93, -21.11, -54.48, -41.13];
y_clean = [-9.32, -55.38, 23.18, 56.88, 39.94, -24.0, -73.43, 37.31, 7.53, 7.51, 7.18, 29.83, -19.59, -1.1, -33.47];
z_clean = [172.86, 172.58, 174.22, 173.57, 173.44, 173.19, 173.32, 173.61, 174.15, 174.45, 174.25, 173.74, 173.59, 174.36, 173.78];

% homepos corrected test
% Neue Messpunkte (gemittelt aus den stabilen Log-Phasen)
x_clean = [-31.04, -41.45, 16.12, 64.71, 47.74, -41.51, 8.09];
y_clean = [-26.79, 33.38, 55.93, -3.34, -71.21, -50.18, -6.41];
z_clean = [ 0.05,   0.49,   0.14,  0.18,   0.16,   0.03,   0.11];


% Neu gemessene und bereinigte Datenpunkte (insgesamt 11 Punkte)
x_clean = [7.6006, -68.0106, -22.9466, 66.1180, 81.4583, 18.2928, -53.9315, -18.5736, -6.9056, 23.6920, 10.8971];
y_clean = [3.4500, 20.9398, 57.6248, 52.3880, -28.5011, -73.8172, -54.5221, -10.3470, 25.0290, 7.8995, -25.5971];
z_clean = [184.7193, 185.7951, 185.7424, 184.5178, 184.7541, 184.4549, 184.6033, 184.9580, 185.5630, 184.9990, 184.4131];

% ohne kompensation 1
% x_clean: Mittelwerte der 12 stabilen Messpunkte
x_clean = [2.2332, -26.6324, 55.7554, 75.8942, 10.7431, -41.4363, ...
           -48.5804, -34.0739, -9.7020, 42.8672, 27.8757, -32.6055];

% y_clean: Mittelwerte der 12 stabilen Messpunkte
y_clean = [5.5716, 64.5050, 52.4087, -18.6811, -72.7663, -55.2600, ...
           31.3850, -23.7193, 30.1449, 6.8327, -63.4981, -23.4523];

% z_clean: Mittelwerte der 12 stabilen Messpunkte
z_clean = [184.9814, 185.9165, 184.9722, 185.5721, 184.8234, 184.4238, ...
           185.4645, 184.8393, 185.4860, 185.4794, 184.8844, 185.1950];

% ohne kompensation 2
% x_clean: Mittelwerte der stabilen Messpunkte (12 Punkte)
x_clean = [3.6221, -19.4477, 56.5499, 73.7225, 11.2144, -43.1425, ...
           -54.3039, -10.3664, 48.2611, 19.4285, -25.1483, -9.5173];

% y_clean: Mittelwerte der stabilen Messpunkte
y_clean = [-3.2906, 57.7403, 34.9489, -17.6987, -79.2081, -46.1557, ...
           25.0975, 34.3409, 7.4086, -42.9989, -32.3912, 26.7600];

% z_clean: Mittelwerte der stabilen Messpunkte
z_clean = [184.8447, 185.9018, 185.0350, 184.7212, 183.9213, 184.2384, ...
           185.5562, 185.6030, 184.8013, 184.1164, 184.5217, 185.2390];


% Mittelwerte der 7 bereinigten Datenblöcke gemessen mit kompensation
% berechnet aus messung ohne kompensation 1
%x_clean = [-2.4023, -10.6887, 53.3344, 82.2342, 24.8815, -38.0601, -56.3119];
%y_clean = [-3.1256, 69.4489, 36.6495, -21.2796, -87.9210, -56.4859, 18.5219];
%z_clean = [-0.5010, 0.6441, 0.4956, -0.9424, -0.8837, 0.1311, 0.2279];

% ohne kompensation 3
% x_clean: Mediane der stabilen Messpunkte (7 Punkte)
x_clean = [6.0853, -28.2335, 62.0862, 82.2225, 16.3139, -46.6503, -59.1126];

% y_clean: Mediane der stabilen Messpunkte
y_clean = [-10.0162, 59.2962, 38.6626, -31.3372, -72.4401, -28.4441, 9.6994];

% z_clean: Mediane der stabilen Messpunkte
z_clean = [184.1044, 185.2273, 184.5774, 183.6637, 183.9374, 185.0489, 185.6723];

% mit kompensation
%x_clean = [-5.0903, -17.4368, 60.0808, 77.6116, 19.1319, -32.8283, -72.2919];
%y_clean = [-2.9772, 56.4873, 39.7592, -22.1440, -71.9203, -40.1771, 28.8692];
%z_clean = [-0.0234, -0.0558, -0.0579, -0.1513, 0.2379, 0.5360, -0.1789];



% ohne kompensation 27.04
%Berechnete Koeffizienten für z = c0 + c1*x + c2*y + c3*x^2 + c4*y^2 + c5*xy:
%  189.7190
%    0.0009
%   -0.0058
%    0.0002
%   -0.0001
%   -0.0002

%Maximaler Fehler VORHER: 2.64 mm
%Maximaler Fehler NACHHER: 0.24 mm
%x_clean = [8.6022, -46.4966, -56.3339, -9.7247, 55.9232, 80.0668, 24.0202];
%y_clean = [2.6463, -44.1872, 25.0292, 70.0830, 44.2765, -27.6395, -70.8381];
%z_clean = [189.7991, 190.1756, 190.1937, 189.2788, 189.4984, 191.9212, 189.9766];



%mit kompensation 27.04
%Berechnete Koeffizienten für z = c0 + c1*x + c2*y + c3*x^2 + c4*y^2 + c5*xy:
%   -0.0246
%    0.0012
%    0.0000
%    0.0003
%    0.0001
%    0.0004

%Maximaler Fehler VORHER: 2.28 mm
%Maximaler Fehler NACHHER: 0.33 mm
%x_clean = [-0.2991, -40.9433, -57.5562, -23.2651, 61.4985, 63.3731, 21.5750, 71.6206];
%y_clean = [6.0486, -42.5696, 32.4113, 58.9688, 39.8392, -5.3330, -64.7807, -6.3552];
%z_clean = [-0.0986, 1.1591, 0.3113, -0.1386, 2.1391, 1.4423, -0.0638, 1.1265];



% ohne kompensation 2 27.04
% Berechnete Koeffizienten für z = c0 + c1*x + c2*y + c3*x^2 + c4*y^2 + c5*xy:
%  189.7740
%    0.0061
%   -0.0010
%    0.0002
%   -0.0001
%   -0.0001
% Maximaler Fehler VORHER: 1.77 mm
% Maximaler Fehler NACHHER: 0.17 mm
%x_clean = [-1.9378, -38.8881, -59.0624, -16.3172, 59.5977, 61.3561, 20.7309];
%y_clean = [0.3983, -52.0232, 27.7789, 73.3294, 46.4177, -23.6100, -51.5816];
%z_clean = [189.7186, 189.1806, 190.0009, 188.8585, 190.1910, 190.6276, 189.8760];

% 1. Design-Matrix aufbauen (Quadratisch: 1, x, y, x^2, y^2, x*y)

x_clean = x_clean(:);
y_clean = y_clean(:);
z_clean = z_clean(:);

A = [ones(length(x_clean), 1), x_clean, y_clean, x_clean.^2, y_clean.^2, x_clean.*y_clean];

% 2. Koeffizienten über kleinste Quadrate fitten (A \ z)
coeffs = A \ z_clean;

c0 = coeffs(1); c1 = coeffs(2); c2 = coeffs(3); 
c3 = coeffs(4); c4 = coeffs(5); c5 = coeffs(6);

disp('Berechnete Koeffizienten für z = c0 + c1*x + c2*y + c3*x^2 + c4*y^2 + c5*xy:');
disp(coeffs);

% 3. So wendest du die Kompensation später im Loop an:
% Wenn deine Kamera z_mess an der Position x_mess, y_mess ausspuckt:
%
% z_modell = c0 + c1*x_mess + c2*y_mess + c3*x_mess^2 + c4*y_mess^2 + c5*x_mess*y_mess;
% z_korrigiert = z_mess - z_modell + c0;  % c0 ist die "echte" Höhe im Zentrum (0,0)

% -- Testen der Kompensation auf die Trainingsdaten --
z_modell_train = A * coeffs;
z_korrigiert_train = z_clean - z_modell_train + c0;

% Fehler vergleichen
max_fehler_vorher = max(z_clean) - min(z_clean);
max_fehler_nachher = max(abs(z_korrigiert_train - c0)); % Abweichung vom Mittelpunkt

fprintf('Maximaler Fehler VORHER: %.2f mm\n', max_fehler_vorher);
fprintf('Maximaler Fehler NACHHER: %.2f mm\n', max_fehler_nachher);
%

% --- SCHUTZSCHALTER ---
%if length(z_clean) < 10
%    error('Fehler: Fast keine Datenpunkte übrig! Bitte prüfe deine Toleranzen.');
%end

% 6. Das 3D-Mesh berechnen und plotten
aufloesung = 100;
xg = linspace(min(x_clean), max(x_clean), aufloesung);
yg = linspace(min(y_clean), max(y_clean), aufloesung);
[X, Y] = meshgrid(xg, yg);
Z = griddata(x_clean, y_clean, z_clean, X, Y, 'natural');
%Z = griddata(x_clean, y_clean, z_korrigiert_train, X, Y, 'natural');

fig_mesh = figure('Name', '3D Fehlerkarte (Entzerrung)');
surf(X, Y, Z, 'EdgeColor', 'none'); 
colormap turbo; 
colorbar;       
title('3D-Karte der Z-Abweichung (Höhenfehler)');
xlabel('X Position (mm)'); 
ylabel('Y Position (mm)'); 
zlabel('Z Abweichung (mm)');
view(3);   
grid on;
axis tight;


% --- 7. Plot des Modells (die glatte gefittete Fläche) ---

% Berechnung der Modell-Werte auf dem gesamten Gitter
% Die Formel entspricht: z = c0 + c1*x + c2*y + c3*x^2 + c4*y^2 + c5*xy
Z_modell_kontinuierlich = c0 + c1.*X + c2.*Y + c3.*X.^2 + c4.*Y.^2 + c5.*X.*Y;

fig_modell = figure('Name', 'Gefittete Modell-Fläche');
% Die glatte Modell-Fläche zeichnen
surf(X, Y, Z_modell_kontinuierlich, 'EdgeColor', 'none', 'FaceAlpha', 0.7); 
hold on;

% Die tatsächlichen Messpunkte (z_clean) als schwarze Punkte einzeichnen
scatter3(x_clean, y_clean, z_clean, 40, 'k', 'filled');

% Die berechneten Modell-Punkte an den Trainingsstellen (z_modell_train) 
% zur Kontrolle als rote Kreise einzeichnen
scatter3(x_clean, y_clean, z_modell_train, 60, 'r', 'LineWidth', 1.5);

colormap turbo;
colorbar;
title('Quadratisches Modell vs. Messdaten');
legend('Gefittetes Modell (Fläche)', 'Messwerte (Training)', 'Modell-Vorhersage an Punkten');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Z (mm)');
view(3);
grid on;


% Das Ideal ist eine perfekt flache Ebene auf der Höhe c0
Z_ideal = ones(size(X)) * c0;

% Die Korrekturfläche ist der Betrag, den du addieren/abziehen musst
Z_korrektur = Z_ideal - Z_modell_kontinuierlich;

fig_corr = figure('Name', 'Kompensations-Fläche (Das Gegenstück)');
surf(X, Y, Z_korrektur, 'EdgeColor', 'none');
colormap(cool); % Nützlich, um zu sehen wo abgezogen (blau) oder addiert (rot) wird
colorbar;
title('Kompensationsprofil (Was auf z_mess addiert wird)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Korrekturwert (mm)');
grid on;
%%
% 7. Ausreißer-Analyse plotten
fig_spikes = figure('Name', 'Ausreißer Analyse (Alle Filter)');

subplot(2, 1, 1);
plot(t, z_fehler, 'b.-'); 
title('Rohdaten (mit Spitzen)');
xlabel('Zeit (s)'); ylabel('Z-Abweichung (mm)');
grid on;

subplot(2, 1, 2);
hold on;
plot(t, z_gefiltert, 'g-', 'LineWidth', 1.5); 

% Wir markieren in ROT alle Punkte, die von IRGENDEINEM der 3 Filter gelöscht wurden
plot(t(idx_alle_fehler), z_fehler(idx_alle_fehler), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); 

title('Gefilterte Daten (Sniper + Physik + Hampel)');
xlabel('Zeit (s)'); ylabel('Z-Abweichung (mm)');
legend('Finale gefilterte Kurve', 'Alle gelöschten Spitzen');
grid on; hold off;


%% Z messung ausserhalb der homepos ebene.

%kinematik command: homepos - 10

%gemessen mit lineal: homepos - 9

%gemessen mit cam gemitttelt über ca. 10 werte:
%homepos - 9.1907


%kinematik command: homepos + 10

%gemessen mit lineal: homepos + 9.5

%gemessen mit cam gemitttelt über ca. 10 werte:
%homepos + 9,6873


% gemessen homepos + 200
%gemessen: 194.795