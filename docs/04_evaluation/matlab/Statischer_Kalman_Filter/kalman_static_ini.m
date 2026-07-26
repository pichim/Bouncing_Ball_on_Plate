clc, clear variables
%% Parameters and model

% Bewegungsgleichung: 3*g*u = 5*ddx  =>  B = 3/5*g
g  = 9810;                      % [mm/s^2]
Ts = 0.001;                     % Abtastzeit [s]

load data_for_sim.mat % save data_for_sim pitch_kin_rad pitch_imu_rad x_ball_mm

time = (0:length(pitch_kin_rad)-1).' * Ts;

figure(1)
subplot(211)
stairs(time, [pitch_kin_rad, pitch_imu_rad]), grid on
subplot(212)
stairs(time, x_ball_mm), grid on

simin = [time, pitch_imu_rad, x_ball_mm];
Tsim = simin(:,1);

%% Static (steady-state) kalman filter

Ts = 1e-3;

A = [[0 1]; [0 0]];
B = [0; 3/5*g];
C = [1 0];
sys = ss(A, B, eye(2), 0);

% Steady-state kalman filter
Q = diag([1 500]);
R = 1e-2;
H = lqr(A.', C.', Q, R).';

% Extend with disturbance input
Ae = [[A, B]; [0, 0, 0]];
Be = [B; 0];
Ce = [C, 0];

% Steady-state kalman filter
Qe = diag([1 500 0.01]);
Re = 1e-2;
He = lqr(Ae.', Ce.', Qe, Re).';

% Closed-Loop EW
eig(A - H*C)
eig(Ae - He*Ce)

% Position sensor runs 10-times slower
Ts_pos = 50 * Ts;
Tt = 0.016;
nd = Tt / Ts

sim('kalman_static_sim.slx');