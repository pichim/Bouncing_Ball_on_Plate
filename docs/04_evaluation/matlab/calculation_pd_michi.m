
%% Michi
% tau = 0.0413; % 1/(2*pi*fcut)
% fcut = 1/(2*pi*tau)

fcut = 2
tau = 1/(2*pi*fcut)

tau_ro = 1/(2*pi*3*fcut)
fcut_ro = 1/(2*pi*tau_ro)

s = tf('s');

C = s / (tau*s + 1);

figure(1)
bode(C, C * 1 / (tau_ro*s + 1)), grid on


%% Luca
% tau = 0.0413; % 1/(2*pi*fcut)
% fcut = 1/(2*pi*tau)

fcut = 2
tau = 1/(2*pi*fcut)

tau_ro = 1/(2*pi*1*fcut)
fcut_ro = 1/(2*pi*tau_ro)

s = tf('s');

C = s / (tau*s + 1);

figure(1)
bode(C, C * 1 / (tau_ro*s + 1)), grid on