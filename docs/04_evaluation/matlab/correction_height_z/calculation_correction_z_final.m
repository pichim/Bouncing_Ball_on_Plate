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

%% X/Y Validation
clear; clc; close all;

% 1. Raw Data (Camera Measurements)
% Index 1 = Center, Indices 2-6 = Outer holes
x_raw = [-0.163, 21.120, -29.205, -39.291, 4.928, 42.192];
y_raw = [0.534, 37.975, 31.918, -17.445, -41.876, -7.922];

% Step A: Centering (Correcting Camera Offset)
x_centered = x_raw - x_raw(1);
y_centered = y_raw - y_raw(1);

% 2. Rotation of Measurements (Internal Compensation)
rot_angle = -60; 
theta_rot = deg2rad(rot_angle);
x_rot = x_centered * cos(theta_rot) - y_centered * sin(theta_rot);
y_rot = x_centered * sin(theta_rot) + y_centered * cos(theta_rot);

% 3. Automated Sorting & Target Definition
R_target_val = 42.5; 
target_angles = 0:72:288;
x_meas_outer = x_rot(2:end);
y_meas_outer = y_rot(2:end);

% Sort measured points by angle to match target sequence (0, 72, 144...)
meas_angles = atan2d(y_meas_outer, x_meas_outer);
meas_angles(meas_angles < -10) = meas_angles(meas_angles < -10) + 360; 
[~, sortIdx] = sort(meas_angles);
x_meas = [0, x_meas_outer(sortIdx)];
y_meas = [0, y_meas_outer(sortIdx)];

% Final Target Coordinates
x_target = [0, R_target_val * cosd(target_angles)];
y_target = [0, R_target_val * sind(target_angles)];

% 4. Error Calculation
dx = x_meas - x_target;
dy = y_meas - y_target;
dr = sqrt(x_meas.^2 + y_meas.^2) - sqrt(x_target.^2 + y_target.^2);

% Statistics
mae_x = mean(abs(dx));
mae_y = mean(abs(dy));
mae_r = mean(abs(dr));
max_r = max(abs(dr));

% 5. Visualization
fig = figure('Color', 'w', 'Name', 'xy Accuracy Evaluation', 'Position', [100, 100, 900, 700]);
hold on; grid on; axis equal;

% Set axes to +/- 60
xlim([-60 60]);
ylim([-60 60]);

plot(x_target, y_target, 'ro', 'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'Target');
plot(x_meas, y_meas, 'bx', 'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'Measured Values');

for i = 1:length(x_target)
    plot([x_target(i) x_meas(i)], [y_target(i) y_meas(i)], 'k:', 'HandleVisibility', 'off');
% Label individual points with Radial Error (using \Delta R for clarity)
text(x_meas(i)+2, y_meas(i)+2, sprintf('\\Delta r: %.2f mm', dr(i)), 'FontSize', 14, 'FontWeight', 'bold');
end

% Titel und Achsenbeschriftungen mit angepassten Schriftgrößen
%title('xy Accuracy Verification', 'FontSize', 22, 'FontWeight', 'bold');
xlabel('x Position [mm]', 'FontSize', 14); 
ylabel('y Position [mm]', 'FontSize', 14);

% Titel erstellen und direkt die Schriftgröße erzwingen
t = title('xy Accuracy Verification', 'FontSize', 18, 'FontWeight', 'bold');

% Falls MATLAB den Titel trotzdem ignoriert, erzwingen wir es hier direkt am Objekt:
%set(t, 'FontSize', 22, 'FontWeight', 'bold');

% Achsenticker vergrößern (Zahlen an den Achsen)
set(gca, 'FontSize', 14);

%title('xy Accuracy Verification');
%xlabel('x Position [mm]'); ylabel('y Position [mm]');
legend('Location', 'northeastoutside','FontSize', 14);

% 6. Detailed Table and Final Statistics Output
fprintf('\n%-8s | %-10s | %-10s | %-10s | %-10s | %-10s\n', 'Hole', 'Target X', 'Target Y', 'Error X', 'Error Y', 'Error R');
fprintf('-------------------------------------------------------------------------------------\n');
for i = 1:6
    fprintf('%-8d | %-10.1f | %-10.1f | %-10.3f | %-10.3f | %-10.3f\n', ...
        i-1, x_target(i), y_target(i), dx(i), dy(i), dr(i));
end
fprintf('-------------------------------------------------------------------------------------\n\n');

fprintf('FINAL STATISTICS\n');
fprintf('--------------------------------------------------\n');
fprintf('Mean Absolute Error X:  %.3f mm\n', mae_x);
fprintf('Mean Absolute Error Y:  %.3f mm\n', mae_y);
fprintf('Mean Absolute Error R:  %.3f mm\n', mae_r);
fprintf('Max Radial Error:       %.3f mm\n', max_r);
fprintf('--------------------------------------------------\n');




%% Doku Z messungen in der ebene

% --- Daten Vorbereitung ---

% Trial 2 (ohne kompensation 2 reduced)
x2 = [3.6221, -19.4477, 56.5499, 73.7225, 11.2144, -43.1425, -54.3039]';
y2 = [-3.2906, 57.7403, 34.9489, -17.6987, -79.2081, -46.1557, 25.0975]';
z2 = [184.8447, 185.9018, 185.0350, 184.7212, 183.9213, 184.2384, 185.5562]';

% Trial 3 (ohne kompensation 3)
x3 = [6.0853, -28.2335, 62.0862, 82.2225, 16.3139, -46.6503, -59.1126]';
y3 = [-10.0162, 59.2962, 38.6626, -31.3372, -72.4401, -28.4441, 9.6994]';
z3 = [184.1044, 185.2273, 184.5774, 183.6637, 183.9374, 185.0489, 185.6723]';

% Gemeinsame Achsenlimits bestimmen für bessere Vergleichbarkeit
all_x = [x2; x3]; all_y = [y2; y3]; all_z = [z2; z3];
x_lims = [min(all_x)-5, max(all_x)+5];
y_lims = [min(all_y)-5, max(all_y)+5];
z_lims = [min(all_z)-0.2, max(all_z)+0.2];
c_lims = [min(all_z), max(all_z)]; % Einheitliche Farbskala

% --- Plotting ---

fig = figure('Name', 'Comparison of Measurement Trials', 'Color', 'w');
% 'TileSpacing','compact' spart Breite/Höhe zwischen den Plots
tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Loop für beide Trials
for i = 1:2
    nexttile;
    if i == 1
        curr_x = x2; curr_y = y2; curr_z = z2;
        t_title = 'Measurement Trial 2';
    else
        curr_x = x3; curr_y = y3; curr_z = z3;
        t_title = 'Measurement Trial 3';
    end
    
    % Mesh generieren
    [X, Y] = meshgrid(linspace(x_lims(1), x_lims(2), 1000), ...
                      linspace(y_lims(1), y_lims(2), 1000));
    Z = griddata(curr_x, curr_y, curr_z, X, Y, 'natural');
    
    % Oberfläche plotten
    surf(X, Y, Z, 'EdgeColor', 'none'); 
    hold on;
    scatter3(curr_x, curr_y, curr_z, 30, 'k', 'filled', 'MarkerEdgeColor', 'w');
    
    % Formatierung
    title(t_title);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    view(3); grid on; axis tight;
    
    % Achsen und Farbe synchronisieren
    xlim(x_lims); ylim(y_lims); zlim(z_lims);
    clim(c_lims); 
    colormap turbo;
end

% Gemeinsame Colorbar rechts
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = 'Z-Deviation (mm)';


%% doku z messungen in ebene genullt
% --- Daten Vorbereitung ---
% Trial 2 (ohne kompensation 2 reduced)
x2 = [3.6221, -19.4477, 56.5499, 73.7225, 11.2144, -43.1425, -54.3039]';
y2 = [-3.2906, 57.7403, 34.9489, -17.6987, -79.2081, -46.1557, 25.0975]';
z2 = [184.8447, 185.9018, 185.0350, 184.7212, 183.9213, 184.2384, 185.5562]';

% Trial 3 (ohne kompensation 3)
x3 = [6.0853, -28.2335, 62.0862, 82.2225, 16.3139, -46.6503, -59.1126]';
y3 = [-10.0162, 59.2962, 38.6626, -31.3372, -72.4401, -28.4441, 9.6994]';
z3 = [184.1044, 185.2273, 184.5774, 183.6637, 183.9374, 185.0489, 185.6723]';

% --- Z "nullen" (Globaler Median) ---
global_median = median([z2; z3]); % Median über alle Messpunkte berechnen

z2 = z2 - global_median; % Median von Trial 2 abziehen
z3 = z3 - global_median; % Median von Trial 3 abziehen

% Gemeinsame Achsenlimits bestimmen für bessere Vergleichbarkeit
all_x = [x2; x3]; all_y = [y2; y3]; all_z = [z2; z3];
x_lims = [min(all_x)-5, max(all_x)+5];
y_lims = [min(all_y)-5, max(all_y)+5];
z_lims = [min(all_z)-0.2, max(all_z)+0.2];
c_lims = [min(all_z), max(all_z)]; % Einheitliche Farbskala

% --- Plotting ---

fig = figure('Name', 'Comparison of Measurement Trials', 'Color', 'w');
% 'TileSpacing','compact' spart Breite/Höhe zwischen den Plots
tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Loop für beide Trials
for i = 1:2
    nexttile;
    if i == 1
        curr_x = x2; curr_y = y2; curr_z = z2;
        t_title = '3D Map of Z-Deviation Measurement 1';
    else
        curr_x = x3; curr_y = y3; curr_z = z3;
        t_title = '3D Map of Z-Deviation Measurement 2';
    end
    
    % Mesh generieren
    [X, Y] = meshgrid(linspace(x_lims(1), x_lims(2), 1000), ...
                      linspace(y_lims(1), y_lims(2), 1000));
    Z = griddata(curr_x, curr_y, curr_z, X, Y, 'natural');
    
    % Oberfläche plotten
    surf(X, Y, Z, 'EdgeColor', 'none'); 
    hold on;
    scatter3(curr_x, curr_y, curr_z, 30, 'k', 'filled', 'MarkerEdgeColor', 'w');
    
    % Formatierung
    title(t_title);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    view(3); grid on; axis tight;
    
    % Achsen und Farbe synchronisieren
    xlim(x_lims); ylim(y_lims); zlim(z_lims);
    clim(c_lims); 
    colormap turbo;
end

% Gemeinsame Colorbar rechts
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = 'Z-Deviation (mm)';


%%
clear; clc; close all;

% --- 1. DATENVORBEREITUNG ---
% Measurement 1 (Trainingsdaten / Fit-Daten)
x1 = [3.6221, -19.4477, 56.5499, 73.7225, 11.2144, -43.1425, -54.3039]';
y1 = [-3.2906, 57.7403, 34.9489, -17.6987, -79.2081, -46.1557, 25.0975]';
z1 = [184.8447, 185.9018, 185.0350, 184.7212, 183.9213, 184.2384, 185.5562]';

% Measurement 2 (Validierungsdaten / Testdaten)
x2 = [6.0853, -28.2335, 62.0862, 82.2225, 16.3139, -46.6503, -59.1126]';
y2 = [-10.0162, 59.2962, 38.6626, -31.3372, -72.4401, -28.4441, 9.6994]';
z2 = [184.1044, 185.2273, 184.5774, 183.6637, 183.9374, 185.0489, 185.6723]';

% --- 2. MODELL-BERECHNUNG (Lineare Ebene fitten mit Meas 1) ---
% Design-Matrix aufbauen: z = c0 + c1*x + c2*y
A_fit = [ones(length(x1), 1), x1, y1];

% Koeffizienten über kleinste Quadrate berechnen
coeffs = A_fit \ z1;
c0 = coeffs(1); 
c1 = coeffs(2); 
c2 = coeffs(3);

fprintf('--- BERECHNETES MODELL ---\n');
fprintf('z = %.4f + %.4f*x + %.4f*y\n\n', c0, c1, c2);

% --- 3. PLOT A: DAS PROBLEM & DAS MODELL (Measurement 1) ---
% Gitter für die Visualisierung der Ausgleichsebene erstellen
aufloesung = 100;
x_grid_lims = [min(x1)-10, max(x1)+10];
y_grid_lims = [min(y1)-10, max(y1)+10];
[X_grid, Y_grid] = meshgrid(linspace(x_grid_lims(1), x_grid_lims(2), aufloesung), ...
                            linspace(y_grid_lims(1), y_grid_lims(2), aufloesung));

% Z-Werte der glatten Modell-Ebene berechnen
Z_modell_ebene = c0 + c1.*X_grid + c2.*Y_grid;

figA = figure('Name', 'Plot A: Measurement 1 + Modell', 'Color', 'w', 'Position', [100, 100, 800, 600]);
% Die berechnete Ausgleichsebene (halbtransparent)
surf(X_grid, Y_grid, Z_modell_ebene, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
colormap turbo; colorbar; hold on;

% Die tatsächlichen, unkorrigierten Messpunkte aus Measurement 1
scatter3(x1, y1, z1, 60, 'k', 'filled', 'MarkerEdgeColor', 'w');

% Formatierung
title('Linear Compensation Plane Fitted to Measurement 1', 'FontSize', 14);
xlabel('X Position (mm)', 'FontSize', 12); 
ylabel('Y Position (mm)', 'FontSize', 12); 
zlabel('Z Coordinate (mm)', 'FontSize', 12);
legend('Fitted Compensation Plane', 'Uncorrected Raw Data', 'Location', 'northeast');
view(-45, 20); % Optimaler Winkel, um die Schräglage zu sehen
grid on; axis tight;

% --- 4. KOMPENSATION ANWENDEN (Auf Null zentriert / Peak-to-Peak) ---
% 1. VORHER: Nur die konstante Referenzhöhe abziehen (Tilt bleibt sichtbar)
z2_vorher_zeroed = z2 - c0;

% 2. NACHHER: Das komplette schiefe Modell abziehen (Messebene wird flach)
z2_modell = c0 + c1.*x2 + c2.*y2;
z2_korrigiert_zeroed = z2 - z2_modell;

% --- STATISTIK: Spannweite (Peak-to-Peak) berechnen ---
% Hier vergleichen wir ehrlich "Tal zu Berg" für VORHER und NACHHER
max_err_vorher_ptp = max(z2_vorher_zeroed) - min(z2_vorher_zeroed);
max_err_nachher_ptp = max(z2_korrigiert_zeroed) - min(z2_korrigiert_zeroed);

fprintf('--- VALIDIERUNG MEASUREMENT 2 (Peak-to-Peak) ---\n');
fprintf('Totale Spannweite VORHER:  %.2f mm\n', max_err_vorher_ptp);
fprintf('Totale Spannweite NACHHER: %.2f mm\n', max_err_nachher_ptp);

% --- 5. PLOT B: DIE VALIDIERUNG (Z-Deviation um 0 mm) ---
% Einheitliche Limits um den Nullpunkt
z_lims = [min([z2_vorher_zeroed; z2_korrigiert_zeroed])-0.2, ...
          max([z2_vorher_zeroed; z2_korrigiert_zeroed])+0.2];
c_lims = [min([z2_vorher_zeroed; z2_korrigiert_zeroed]), ...
          max([z2_vorher_zeroed; z2_korrigiert_zeroed])];

figB = figure('Name', 'Plot B: Validierung Measurement 2 (Zeroed)', 'Color', 'w', 'Position', [150, 150, 1000, 500]);
tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% SUBPLOT 1: VORHER (Genullt, aber noch schief)
nexttile;
Z_vorher_mesh = griddata(x2, y2, z2_vorher_zeroed, X_grid, Y_grid, 'natural');
surf(X_grid, Y_grid, Z_vorher_mesh, 'EdgeColor', 'none'); hold on;
scatter3(x2, y2, z2_vorher_zeroed, 40, 'k', 'filled', 'MarkerEdgeColor', 'w');

title('Measurement 2: Uncorrected Deviation', 'FontSize', 12);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z Deviation (mm)');
view(-45, 20); grid on; axis tight;
zlim(z_lims); clim(c_lims); colormap turbo;

% SUBPLOT 2: NACHHER (Genullt und flach)
nexttile;
Z_nachher_mesh = griddata(x2, y2, z2_korrigiert_zeroed, X_grid, Y_grid, 'natural');
surf(X_grid, Y_grid, Z_nachher_mesh, 'EdgeColor', 'none'); hold on;
scatter3(x2, y2, z2_korrigiert_zeroed, 40, 'k', 'filled', 'MarkerEdgeColor', 'w');

title('Measurement 2: Compensated Deviation', 'FontSize', 12);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z Deviation (mm)');
view(-45, 20); grid on; axis tight;
zlim(z_lims); clim(c_lims); colormap turbo;

% Gemeinsame Colorbar
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = 'Z Deviation (mm)';
cb.Label.FontSize = 12;

%% mit linkpro p (gleiche settings für linken und rechten plot)
% --- 5. PLOT B: DIE VALIDIERUNG (Z-Deviation um 0 mm) ---
% Einheitliche Limits um den Nullpunkt
z_lims = [min([z2_vorher_zeroed; z2_korrigiert_zeroed])-0.2, ...
          max([z2_vorher_zeroed; z2_korrigiert_zeroed])+0.2];
c_lims = [min([z2_vorher_zeroed; z2_korrigiert_zeroed]), ...
          max([z2_vorher_zeroed; z2_korrigiert_zeroed])];

figB = figure('Name', 'Plot B: Validierung Measurement 2 (Zeroed)', 'Color', 'w', 'Position', [150, 150, 1000, 500]);
tlo = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% SUBPLOT 1: VORHER (Genullt, aber noch schief)
ax1 = nexttile; % <-- WICHTIG: Handle in 'ax1' speichern
Z_vorher_mesh = griddata(x2, y2, z2_vorher_zeroed, X_grid, Y_grid, 'natural');
surf(X_grid, Y_grid, Z_vorher_mesh, 'EdgeColor', 'none'); hold on;
scatter3(x2, y2, z2_vorher_zeroed, 40, 'k', 'filled', 'MarkerEdgeColor', 'w');
title('Measurement 2: Uncorrected Deviation', 'FontSize', 12);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z Deviation (mm)');
view(-45, 20); grid on; axis tight;
zlim(z_lims); clim(c_lims); colormap turbo;

% SUBPLOT 2: NACHHER (Genullt und flach)
ax2 = nexttile; % <-- WICHTIG: Handle in 'ax2' speichern
Z_nachher_mesh = griddata(x2, y2, z2_korrigiert_zeroed, X_grid, Y_grid, 'natural');
surf(X_grid, Y_grid, Z_nachher_mesh, 'EdgeColor', 'none'); hold on;
scatter3(x2, y2, z2_korrigiert_zeroed, 40, 'k', 'filled', 'MarkerEdgeColor', 'w');
title('Measurement 2: Compensated Deviation', 'FontSize', 12);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z Deviation (mm)');
view(-45, 20); grid on; axis tight;
zlim(z_lims); clim(c_lims); colormap turbo;

% Gemeinsame Colorbar
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = 'Z Deviation (mm)';
cb.Label.FontSize = 12;

% --- NEU: SYNCHRONISATION DER ROTATION ---
% Verknüpft die View-Eigenschaft beider Achsen
hLink = linkprop([ax1, ax2], 'View');

% WICHTIG: Das Verknüpfungsobjekt (hLink) darf nicht gelöscht werden.
% Am sichersten ist es, dieses Objekt an die Figure anzuhängen.
setappdata(figB, 'RotationLink', hLink);


