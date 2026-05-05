clc, clear variables
%% parameters and models

% Maxon Steller max 50V, 5A

k = 15.37;   % force constant (N/A)
r = 0.445;   % radius between coils and pivot (m)
l = 0.04;    % distance to com (m)
J = 0.213;   % inertia beam (kgm^3)
m = 2.744;   % mass beam (kg)
g = 9.81;    % gravity (m/s^2)
i_max = 0.4; % max desired current (A)

% model plant: angle -> ball position
G_pos_mod = tf(5*g, [7 0 0]);


%% ball position with controlSystemDesigner

% - fine tuning of the hole controller
load Cx_4.mat % save Cx_4 Cx_4
% 37.186 (s+0.4) (s+0.5)
% ----------------------
%     s (s+6) (s+20)

figure(1)
bode(Cx_4), grid on
title('Position Controller')
legend('Location', 'best')

Cx = Cx_4


%% setpoint filter of ball position

Gcl_x = feedback(Cx * G_pos_mod, 1);

Tf = 1.0;
F = tf(1, [Tf, 1]); % 1 / (Tf*s + 1)

figure(2)
step(Gcl_x, F * Gcl_x), grid on


%% static (steady-state) kalman filter

Ts = 1e-3;
Cxd = c2d(Cx, Ts, 'tustin');
Fd  = c2d(F , Ts, 'tustin');

% 5*g * u = 7 * ddx
A = [[0 1]; [0 0]];
B = [0; 5/7*g];
C = [1 0];
% D = 0;

% euler discretization, could also be zoh
A = eye(size(A)) + Ts * A;
B = Ts * B;

% static kalman-filter
R = 0.1 / Ts; % tune here
Q = B * B.' * 1 * Ts;
K = dlqr(A.', C.', Q, R).';

% extend with disturbance input
% Tg = 1 / (2*pi*300);
% Ae = [[A, B]; [0, 0, (1 - Ts/Tg)]];
Ae = [[A, B]; [0, 0, 1]];
Be = [B; 0];
Ce = [C, 0];

% static kalman-filter for with input disturbance estimator
R  = 0.1 / Ts; % tune here
Qe = Be * Be.' * 1 * Ts;
Qe(3,3) = 1e-1; % you need to add the penalty for the disturbance extra!
Ke = dlqr(Ae.', Ce.', Qe, R).';

% - you can also formulate output disturbances
% - you can also formulate the disturbance dynamics not as integrator but
%   as a lowpass type, this sometimes helps with observability
