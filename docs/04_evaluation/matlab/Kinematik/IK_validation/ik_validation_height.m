clc, clear variables
close all

set(groot, 'DefaultFigureWindowStyle', 'docked');

soll = [85.5 90.5 100.5 110.5 120.5 130.5 135.5 140.5];
ist1 = [141.0 145.0 153.0 161.0 171.0 182.0 189.0 195.0] - ones(1,8) * 50.5;
ist2 = [140.0 144.0 152.0 161.0 171.0 182.5 188.5 195.0] - ones(1,8) * 50.5;
ist3 = [140.5 144.0 152.0 161.0 171.0 181.5 188.0 194.5] - ones(1,8) * 50.5;

ist = (ist1 + ist2 + ist3) / 3;

err = soll - ist;

figure(1)
plot(soll, err, 'o', 'LineWidth', 1.5, 'MarkerSize', 7)
yline(0, '--', 'LineWidth', 1.0)

title('Height error at the three plate points')
xlabel('Desired height [mm]')
ylabel('Height error [mm]')

grid on

%% Code von ChatGPT für den fit:

% Linearer Fit (Grad 1)
p = polyfit(soll, err, 2);          % Koeffizienten berechnen
soll_fit = linspace(min(soll), max(soll), 100);
err_fit = polyval(p, soll_fit);     % Fit auswerten

hold on
plot(soll_fit, err_fit, '-r', 'LineWidth', 1.5)
legend('Measured error', 'zero', 'fit', 'Location', 'best')

fprintf('Slope: %.4f mm/mm\n', p(1))
fprintf('Offset: %.4f mm\n', p(2))