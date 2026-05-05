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

% Cx = Cx_4
s = tf('s');
Gr = 37.186 / 20 *(s+0.4)*(s+0.5) / ( s*(s+6) ) % no roll-off filter
% Gr = 37.186*(s+0.4)*(s+0.5) / ( s*(s+6)*(s+20) )

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

% euler discretization, could also be zoh
A = eye(size(A)) + Ts * A;
B = Ts * B;

% % zoh
% M = expm([A, B; zeros(1,3)] * Ts);
% A = M(1:2,1:2);
% B = M(1:2,3);

% static kalman-filter
R = 0.1 / Ts; % tune here
% Q = B * B.' * 1 * Ts;
Q = diag([1 0.01]);
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
