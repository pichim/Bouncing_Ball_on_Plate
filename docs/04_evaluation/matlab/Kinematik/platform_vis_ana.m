clear; clc; close all;

% =========================================================================
% ANALYTICAL INVERSE KINEMATICS FOR THE 3-LEG PLATFORM
% =========================================================================
%
% PURPOSE
% -------
% This script computes the 3 motor angles analytically from:
%
%   roll, pitch, h
%
% where
%   roll  = platform rotation about x
%   pitch = platform rotation about y
%   h     = platform-center height
%
% The script assumes:
%   1) 3 identical legs arranged symmetrically at 120 degrees
%   2) each leg moves in its own vertical plane
%   3) each leg consists of:
%        - a servo horn of length r1
%        - a rod of length r2
%        - a ball joint on the platform
%
% IMPORTANT MODEL IDEA
% --------------------
% The desired platform pose is not used directly.
%
% First, the platform is tilted by roll and pitch.
% Then an additional rigid-body correction is computed analytically:
%
%   - translation in x
%   - translation in y
%   - yaw rotation about z
%
% This correction makes the 3 platform joints lie exactly in the 3 actuator
% planes. After that, each leg becomes a simple 2D inverse-kinematics
% problem.
%
% BRANCH SELECTION
% ----------------
% Each leg has two mathematical angle solutions.
% In this simplified script, we only allow the branch where the SERVO HORN
% points outward.
%
% If both candidates point outward, we choose the one with the smaller
% wrapped angle in [0, 360).
%
% If neither candidate points outward, the requested pose is rejected.
%
% ANGLE CONVENTION
% ----------------
% In the local leg frame:
%
%   alpha = 0 deg   -> horn points straight downward
%   alpha = 90 deg  -> horn points horizontally outward
%   alpha = 180 deg -> horn points straight upward
%
% =========================================================================
% USER INPUT
% =========================================================================

% Plattenwinkel und Höhe
roll = deg2rad(0.0);               % [rad]
pitch = deg2rad(0.0);              % [rad]
h = 110.5;                          % [mm] von Servowelle-Ebene zu Kugelgelenk-Ebene

% =========================================================================
% ROTATION MATRICES
% =========================================================================

Rx = @(phi) [1 0 0; ...
             0 cos(phi) -sin(phi); ...
             0 sin(phi)  cos(phi)];

Ry = @(phi) [cos(phi) 0 sin(phi); ...
              0        1 0; ...
             -sin(phi) 0 cos(phi)];

Rz = @(phi) [cos(phi) -sin(phi) 0; ...
             sin(phi)  cos(phi) 0; ...
             0         0        1];

% =========================================================================
% GEOMETRY
% =========================================================================
%
% TH = azimuth angles of the 3 motors
% r0 = radius of motor axes from the center
% r1 = servo-horn length
% r2 = rod length
% rp = radius of plate
%
% M0 = motor centers
% B0 = horn tips for alpha = 0 deg
% P0 = neutral platform joints

TH = pi/180 * [0 120 240];
r0 = 68.0;
r1 = 85.0;
r2 = 117.0;
rp = 215/2;

M0 = [r0*[cos(TH); sin(TH)];  0  0  0];
B0 = [r0*[cos(TH); sin(TH)]; -r1 -r1 -r1];
P0 = [(rp)*[cos(TH); sin(TH)]; 0 0 0];

% =========================================================================
% STEP 1: TILT THE PLATFORM BY ROLL AND PITCH
% =========================================================================
%
% This gives the desired platform shape before the hidden x/y/yaw correction.

P_tilt = Ry(pitch) * Rx(roll) * P0;

% =========================================================================
% STEP 2: ANALYTICAL PLATFORM LOCKING
% =========================================================================
%
% We seek:
%
%   Pcorr = [dx; dy; h] + Rz(yaw) * P_tilt
%
% such that platform joint i lies in actuator plane i.
%
% For the symmetric 120-degree geometry, dx, dy, yaw can be obtained
% analytically.

a = zeros(3,1);
b = zeros(3,1);

for i = 1:3
    x_i = P_tilt(1,i);
    y_i = P_tilt(2,i);

    % radial / tangential projections of the tilted point
    a(i) =  x_i*cos(TH(i)) + y_i*sin(TH(i));
    b(i) = -x_i*sin(TH(i)) + y_i*cos(TH(i));
end

A = sum(a);
B = sum(b);

% yaw follows from:
%   A*sin(yaw) + B*cos(yaw) = 0
yaw = atan2(-B, A);

f = a*sin(yaw) + b*cos(yaw);

% for TH = [0, 120, 240] deg:
dy = -f(1);
dx = (f(2) - f(3)) / sqrt(3);

Pcorr = [dx; dy; h] + Rz(yaw) * P_tilt;
C = mean(Pcorr, 2);

% =========================================================================
% BALL POSITION IN GLOBAL FRAME
% =========================================================================
ball_radius = 20.0;     % [mm], z.B. Pingpongball: 40 mm Durchmesser
plate_offset = 0.0;    % [mm], Platte liegt 11.5 mm über der Kugelgelenk-Ebene

% =========================================================================
% STEP 3: ANALYTICAL 2D IK FOR EACH LEG
% =========================================================================
%
% For leg i:
%   1) rotate platform joint i into the local leg frame
%   2) solve the planar 2-link problem analytically
%   3) keep only the branch where the horn points outward
%
% In the local leg frame:
%
%   motor axis = [r0; 0; 0]
%
%   horn tip   = [r0 + r1*sin(alpha);
%                 0;
%                -r1*cos(alpha)]
%
% The corrected platform joint should lie in the local x-z plane.

alpha          = nan(3,1);    % chosen angle [rad], raw branch value
alpha_deg      = nan(3,1);    % chosen angle [deg], wrapped to [0,360)
alpha_cand_deg = nan(3,2);    % both candidate angles [deg], wrapped
H_local        = nan(3,3);    % chosen horn tip in local frame
H_global       = nan(3,3);    % chosen horn tip in global frame

plane_residual = nan(3,1);    % local y of corrected platform point
rod_error      = nan(3,1);    % rod length error
horn_outward   = false(3,1);  % chosen horn points outward?

tol = 1e-12;

for i = 1:3

    % ---------------------------------------------------------------------
    % 3.1 Corrected platform joint in local leg frame
    % ---------------------------------------------------------------------
    P_local = Rz(-TH(i)) * Pcorr(:,i);
    plane_residual(i) = P_local(2);

    % ---------------------------------------------------------------------
    % 3.2 Target relative to motor axis
    % ---------------------------------------------------------------------
    %
    % Motor axis: [r0; 0; 0]
    % Platform point: [x; 0; z]
    %
    % Define:
    %   u = x - r0
    %   w = z
    %
    % Then rho is the distance from the motor axis to the platform point.

    u   = P_local(1) - r0;
    w   = P_local(3);
    rho = hypot(u, w);

    % reachability of the 2-link chain
    if rho < abs(r1-r2) - tol || rho > (r1+r2) + tol
        error('Leg %d is not reachable for the requested pose.', i);
    end

    % ---------------------------------------------------------------------
    % 3.3 Two analytical IK branches
    % ---------------------------------------------------------------------
    %
    % The rod-length equation leads to:
    %
    %   alpha = beta +/- gamma
    %
    % where
    %   beta  = atan2(u, -w)
    %   gamma = acos((r1^2 + rho^2 - r2^2)/(2*r1*rho))

    c = (r1^2 + rho^2 - r2^2) / (2*r1*rho);
    c = max(-1, min(1, c));   % numerical safety

    beta  = atan2(u, -w);
    gamma = acos(c);

    cand = [beta + gamma; ...
            beta - gamma];

    Hcand = zeros(3,2);
    cand_deg_wrap = zeros(2,1);
    is_horn_outward = false(2,1);

    for k = 1:2
        % horn tip for candidate k
        Hcand(:,k) = [r0 + r1*sin(cand(k)); ...
                      0; ...
                     -r1*cos(cand(k))];

        % wrapped angle in [0, 360)
        cand_deg_wrap(k) = mod(rad2deg(cand(k)), 360);

        % outward means: horn tip lies radially outside the motor axis
        is_horn_outward(k) = (Hcand(1,k) >= r0 - tol);
    end

    alpha_cand_deg(i,:) = cand_deg_wrap.';

    % ---------------------------------------------------------------------
    % 3.4 Keep only outward-horn branches
    % ---------------------------------------------------------------------
    ok = find(is_horn_outward);

    if isempty(ok)
        error('Leg %d has no outward-horn solution for this pose.', i);
    end

    % If both are outward, choose the smaller wrapped angle.
    if numel(ok) == 2
        [~, idx] = min(cand_deg_wrap(ok));
        chosen = ok(idx);
    else
        chosen = ok(1);
    end

    alpha(i)     = cand(chosen);
    alpha_deg(i) = cand_deg_wrap(chosen);
    H_local(:,i) = Hcand(:,chosen);
    H_global(:,i)= Rz(TH(i)) * H_local(:,i);

    horn_outward(i) = is_horn_outward(chosen);

    % rod-length check
    rod_error(i) = norm(P_local - H_local(:,i)) - r2;
end

% =========================================================================
% TEXT OUTPUT
% =========================================================================

fprintf('\n============================================================\n');
fprintf('ANALYTICAL INVERSE KINEMATICS RESULT\n');
fprintf('============================================================\n');

fprintf('\nDesired pose:\n');
fprintf('  roll  = %+8.4f rad  = %+8.3f deg\n', roll,  rad2deg(roll));
fprintf('  pitch = %+8.4f rad  = %+8.3f deg\n', pitch, rad2deg(pitch));
fprintf('  h     = %+8.4f\n', h);

fprintf('\nHidden analytical correction:\n');
fprintf('  dx   = %+12.6f\n', dx);
fprintf('  dy   = %+12.6f\n', dy);
fprintf('  yaw  = %+12.6f rad  = %+9.4f deg\n', yaw, rad2deg(yaw));

fprintf('\nPlatform center after correction:\n');
fprintf('  C = [%+.6f  %+.6f  %+.6f]^T\n', C(1), C(2), C(3));

fprintf('\nMotor angles:\n');
for i = 1:3
    fprintf('  Leg %d:\n', i);
    fprintf('    candidate 1 = %+10.5f deg\n', alpha_cand_deg(i,1));
    fprintf('    candidate 2 = %+10.5f deg\n', alpha_cand_deg(i,2));
    fprintf('    chosen      = %+10.5f deg\n', alpha_deg(i));
end

fprintf('\nChecks:\n');
for i = 1:3
    fprintf('  Leg %d:\n', i);
    fprintf('    plane residual y_local = %+ .3e\n', plane_residual(i));
    fprintf('    rod length error       = %+ .3e\n', rod_error(i));
    fprintf('    horn points outward?   = %d\n', horn_outward(i));
end

fprintf('\nSummary:\n');
fprintf('  max abs plane residual = %.3e\n', max(abs(plane_residual)));
fprintf('  max abs rod error      = %.3e\n', max(abs(rod_error)));
fprintf('  all outward?           = %d\n', all(horn_outward));

% =========================================================================
% SIMPLE 3D PLOT
% =========================================================================

figure('Color','w','Name','Analytical inverse kinematics');
hold on; grid on; axis equal; view(3);

xlabel('x');
ylabel('y');
zlabel('z');
title('3-leg platform: analytical inverse kinematics');

% draw actuator planes
zmin = min([Pcorr(3,:) H_global(3,:) B0(3,:)]) - 20;
zmax = max([Pcorr(3,:) H_global(3,:) B0(3,:)]) + 20;
s1 = 40;
s2 = 130;

for i = 1:3
    er = [cos(TH(i)); sin(TH(i)); 0];
    Cplane = [s1*er + [0;0;zmin], ...
              s2*er + [0;0;zmin], ...
              s2*er + [0;0;zmax], ...
              s1*er + [0;0;zmax]];

    patch(Cplane(1,:), Cplane(2,:), Cplane(3,:), [0.8 0.8 0.8], ...
        'FaceAlpha', 0.5, 'EdgeColor', 'k');
end

% motor centers
plot3(M0(1,:), M0(2,:), M0(3,:), 'ks', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'motor centers');

% neutral horn tips, just as reference
plot3(B0(1,:), B0(2,:), B0(3,:), 'kv', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 7, 'DisplayName', 'neutral horn tips');

% corrected platform triangle
draw_triangle(Pcorr, 'b-', 'corrected platform');

% platform center
plot3(C(1), C(2), C(3), 'bo', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 7, 'DisplayName', 'platform center');

% servo horns and rods
for i = 1:3
    % motor center -> horn tip
    plot3([M0(1,i) H_global(1,i)], ...
          [M0(2,i) H_global(2,i)], ...
          [M0(3,i) H_global(3,i)], 'r-', ...
          'LineWidth', 2, 'HandleVisibility', 'off');

    % horn tip -> platform joint
    plot3([H_global(1,i) Pcorr(1,i)], ...
          [H_global(2,i) Pcorr(2,i)], ...
          [H_global(3,i) Pcorr(3,i)], 'g-', ...
          'LineWidth', 2, 'HandleVisibility', 'off');

    % markers
    plot3(H_global(1,i), H_global(2,i), H_global(3,i), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 7, ...
        'HandleVisibility', 'off');

    plot3(Pcorr(1,i), Pcorr(2,i), Pcorr(3,i), 'bo', ...
        'MarkerFaceColor', 'b', 'MarkerSize', 7, ...
        'HandleVisibility', 'off');

    text(Pcorr(1,i), Pcorr(2,i), Pcorr(3,i), sprintf('  %d', i), ...
        'FontSize', 11, 'Color', 'b');
end

legend('Location','bestoutside');
sgtitle(sprintf('roll = %.2f deg, pitch = %.2f deg, h = %.2f', ...
    rad2deg(roll), rad2deg(pitch), h));

% =========================================================================
% LOCAL HELPER FUNCTION
% =========================================================================

function draw_triangle(P, style, name)
    idx = [1 2 3 1];
    plot3(P(1,idx), P(2,idx), P(3,idx), style, ...
        'LineWidth', 2, 'DisplayName', name);
end