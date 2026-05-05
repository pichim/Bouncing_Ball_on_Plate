clc, clear variables
%% Parameters and model

k = 15.37;   % force constant (N/A)
r = 0.445;   % radius between coils and pivot (m)
l = 0.04;    % distance to com (m)
J = 0.213;   % inertia beam (kgm^3)
m = 2.744;   % mass beam (kg)
g = 9.81;    % gravity (m/s^2)
i_max = 0.4; % max desired current (A)

% model plant: angle -> ball position
G = tf(5*g, [7 0 0]);


%% Ball position controller

s = tf('s');
% Gr = 37.186 / 20 *(s+0.4)*(s+0.5) / ( s*(s+6) ) % no roll-off filter
Gr = 37.186*(s+0.4)*(s+0.5) / ( s*(s+6)*(s+20) )

figure(1)
bode(Gr), grid on
title('Position Controller')
legend('Location', 'best')


%% Setpoint filter of ball position

Gcl = feedback(Gr * G, 1);

Tf = 1.0;
F = tf(1, [Tf, 1]); % 1 / (Tf*s + 1)

figure(2)
step(Gcl, F * Gcl), grid on


%% Static (steady-state) kalman filter

Ts = 2e-3;

% 5*g * u = 7 * ddx
A = [[0 1]; [0 0]];
B = [0; 5/7*g];
C = [1 0];
% D = 0;

% Extend with disturbance input
Ae = [[A, B]; [0, 0, 0]];
Be = [B; 0];
Ce = [C, 0];

% Steady-state kalman filter
Q = diag([1 0.1]);
R = 1e-2;
H = lqr(A.', C.', Q, R).';

% Steady-state kalman filter
Q(3,3) = 10;
He = lqr(Ae.', Ce.', Q, R).';

% Closed-Loop EW
eig(A - H*C)
eig(Ae - He*Ce)

% Position sensor runs 10-times slower
Ts_pos = 10 * Ts;
Tt = 0.02;
nd = Tt / Ts

% d/dt-filter
G_DT1   = c2d(tf([1 0], [1/(2*pi*1) 1]), Ts, 'tustin');

% - you can also formulate output disturbances
% - you can also formulate the disturbance dynamics not as integrator but
%   as a lowpass type, this sometimes helps with observability
