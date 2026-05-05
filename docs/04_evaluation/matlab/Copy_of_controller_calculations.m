clc, clear all
%%

% static constexpr float BALL_CTRL_KP = 0.2383 * 0.3f; //0.2f;
% static constexpr float BALL_CTRL_KI = 0.0f;
% static constexpr float BALL_CTRL_TAU_V = 4.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
% static constexpr float BALL_CTRL_TAU_f = 0.1314f * 2.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
% static constexpr float BALL_CTRL_TAU_R_O = 0.0398f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
% static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
% static constexpr float VISION_TIMEOUT = 1.0F; // seconds

KP = 0.2383 * 0.3;
KI = 0.0;
TAU_V = 4.5;
TAU_F = 0.1314 * 2.5;
TAU_RO = 0.0398 * 5.0;
KD = KP * (TAU_V - TAU_F)
% Ts = 1/50;
% C1 = pid(KP, KI, KD, TAU_F, Ts, ...
%     'IFormula', 'BackwardEuler', ...
%     'DFormula', 'Trapezoidal') * ...
%     c2d(tf(1, [TAU_RO 1]), Ts, 'tustin');
C1 = pid(KP, KI, KD, TAU_F) * ...
    tf(1, [TAU_RO 1]);

KP = 0.5 * 0.8 * 0.2383 * 0.3
KI = 0.0
% TAU_V = 4.5
% TAU_F = 0.1314 * 2.5
TAU_RO = 1 / (2*pi*0.8)
KD = 0.5 * 0.8 * 0.29
TAU_V = KD/KP +  TAU_F
% Ts = 1/50;
C2 = pid(KP, KI, KD, TAU_F) * ...
    tf(1, [TAU_RO 1]);

figure(1)
bode(C1, C2), grid on
