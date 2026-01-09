clc, clear variables
%%

fs = 500;         % Sampling frequency (Hz), must be multiple of 15
f0 = 16;          % Fundamental notch frequency (Hz)
N  = round(fs/f0) % Filter length

% Design the FIR (moving-average) filter
b = (1/N)*ones(1, N); % FIR coefficients
a = 1;                % Denominator for an FIR filter

% Verify DC Gain
dc_gain = sum(b) % Should be 1

Ts = 1/fs;

Gavg = tf(b, eye(size(b)), Ts);

% IIR-Filter
fn = 16;
Dn = 0.2;
Gf = get_notch(fn, Dn, Ts)
Gf = Gf * get_lowpass2(10, 1, Ts);

% Plot frequency response
figure(77)
bode(Gavg, Gf, 2*pi*logspace(-1, log10(1/(2*Ts)), 1e3)), grid on
title('Frequency Response of the Notching Comb FIR Filter');
