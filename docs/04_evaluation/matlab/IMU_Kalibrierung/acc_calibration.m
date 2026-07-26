clc; clear;

ind.acc = 4:6;

files = {
    'acc_z_up.mat'
    'acc_z_down.mat'
    'acc_x_up.mat'
    'acc_x_down.mat'
    'acc_y_up.mat'
    'acc_y_down.mat'
    'acc_tilt_1.mat'
    'acc_tilt_2.mat'
    'acc_tilt_3.mat'
};

acc_means = zeros(numel(files), 3);

for k = 1:numel(files)
    S = load(files{k}, 'data');
    acc_means(k, :) = mean(S.data.values(:, ind.acc), 1);
end

acc_means

%%
norm_before = sqrt(sum(acc_means.^2, 2));

magcal_option_str = 'auto';   % alternativ später 'sym' oder 'diag'
[A, b, expmfs] = magcal(acc_means, magcal_option_str);

acc_means_cal = (acc_means - b) * A;
norm_after = sqrt(sum(acc_means_cal.^2, 2));

A
b
expmfs
norm_before
norm_after

figure(10); clf
subplot(1,2,1)
plot3(acc_means(:,1), acc_means(:,2), acc_means(:,3), '.', 'MarkerSize', 20), grid on, axis equal
title('Before calibration')

subplot(1,2,2)
plot3(acc_means_cal(:,1), acc_means_cal(:,2), acc_means_cal(:,3), '.', 'MarkerSize', 20), grid on, axis equal
title('After calibration')