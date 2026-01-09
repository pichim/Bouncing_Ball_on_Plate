clc, clear all
%%

% measurements static (Navg = 33 @ fs 200Hz)
% file_id = fopen('004.bin'); % ~30 sec measured with pc as power, 22 signals
% file_id = fopen('012.bin'); % ~30 sec measured with one battery pack, 4 signals
% file_id = fopen('015.bin'); % ~30 sec measured with two battery packs, 4 signals
% file_id = fopen('018.bin'); % ~30 sec measured with two battery packs, 22 signals
% file_id = fopen('026.bin'); % ~30 sec measured with two battery packs, 22 signals (dark in room)

% measurements with movement (Navg = 33 @ fs 200Hz)
% file_id = fopen('021.bin'); % ~30 sec measured with two battery packs, 21 signals

% measurements with movement (Navg = 31 @ fs 200Hz)
% file_id = fopen('024.bin'); % ~30 sec measured with two battery packs, 21 signals

% file_id = fopen('029.bin'); % ~30 sec measured with two battery packs, 21 signals

% file_id = fopen('056.bin'); % test measurement
file_id = fopen('002.bin'); % test measurement

num_of_floats = fread(file_id, 1, 'uint8')

data_raw = fread(file_id, 'single');
length(data_raw)

fclose(file_id);


%%

data_raw = data_raw(1:floor( length(data_raw)/num_of_floats ) * num_of_floats);

data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
            
data.time = cumsum(data.values(:,1)) * 1e-6;
data.time = data.time - data.time(1);

data.values = data.values(:,2:end);

size(data.values)


%%

% load data_00.mat % measured via serial stream

Ts = mean(diff(data.time));

% % Offline IIR-Filter
% fn = 16;
% Dn = 0.2;
% Gf = get_notch(fn, Dn, Ts)
% Gf = Gf * get_lowpass2(10, 1, Ts);
% 
% yf = lsim(Gf, data.values(:,2), data.time);

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])

figure(2)
ax(1) = subplot(121);
plot(data.time, data.values(:,1)), grid on
ax(2) = subplot(122);
plot(data.time, [data.values(:,2), data.values(:,3)]), grid on
% plot(data.time, [data.values(:,2), data.values(:,3), yf]), grid on
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])

figure(3)
pwelch([data.values(:,1:3)], [], [], [], 1/Ts), grid on
% pwelch([data.values(:,1:3), yf], [], [], [], 1/Ts), grid on

