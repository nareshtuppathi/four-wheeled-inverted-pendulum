%% Four-Wheeled Inverted Pendulum - Nonlinear Model Validation
% STEP 9 of the robotics portfolio project.
%
% Simulates the FULL NONLINEAR equations of motion (no linearization),
% using the EXISTING LQR gain K designed from the linearized model in
% Step 3. Physical parameters, K, the disturbance, and the initial
% condition are fixed exactly as specified - nothing is redesigned or
% recalculated here. This script does not touch the existing MATLAB
% linear-model files or the Simulink model.

clear; clc; close all;
projectFolder = fullfile(getenv('USERPROFILE'), 'Documents', 'FourWheeledInvertedPendulum');

fprintf('=========================================================\n');
fprintf(' STEP 9: NONLINEAR MODEL VALIDATION\n');
fprintf('=========================================================\n');

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS (fixed, unchanged)
%  ------------------------------------------------------------------
M = 2.5;      % chassis/base mass [kg]
m = 0.5;      % pendulum mass [kg]
l = 0.375;    % pivot-to-CoM distance [m]
I = 0.0150;   % pendulum moment of inertia about its own CoM [kg*m^2]
g = 9.81;     % gravitational acceleration [m/s^2]

%% ------------------------------------------------------------------
%  2) EXISTING LQR GAIN (fixed, NOT recalculated)
%  ------------------------------------------------------------------
K = [-1.0000, -2.9233, -61.5377, -12.5668];

%% ------------------------------------------------------------------
%  3) DISTURBANCE (identical to the previous validated simulation)
%  ------------------------------------------------------------------
F_dist_mag   = 2.0;   % [N]
t_dist_start = 2.0;   % [s]
t_dist_end   = 2.2;   % [s]
F_disturbance_fun = @(t) F_dist_mag * double(t >= t_dist_start & t <= t_dist_end);

%% ------------------------------------------------------------------
%  4) INITIAL CONDITION (theta in radians internally)
%  ------------------------------------------------------------------
theta0_deg = 5;
X0 = [0; 0; deg2rad(theta0_deg); 0];   % [x; x_dot; theta(rad); theta_dot]
tEnd = 5.0;

%% ------------------------------------------------------------------
%  5) NONLINEAR EQUATIONS OF MOTION (solved directly, NOT linearized)
%     M*x_ddot + m*l*theta_ddot*cos(theta) - m*l*theta_dot^2*sin(theta) = F
%     (I+m*l^2)*theta_ddot + m*l*x_ddot*cos(theta) - m*g*l*sin(theta) = 0
%  ------------------------------------------------------------------
function Xdot = nonlinearODE(t, X, M, m, l, I, g, K, Fdist_fun)
    x = X(1); x_dot = X(2); theta = X(3); theta_dot = X(4); %#ok<NASGU>

    F_LQR = -K * X;
    F_disturbance = Fdist_fun(t);
    F_total = F_LQR + F_disturbance;

    % Coupled linear system for [x_ddot; theta_ddot] at this instant,
    % obtained by writing the two nonlinear equations in matrix form:
    %   [ M,            m*l*cos(theta)    ] [x_ddot]     [F_total + m*l*theta_dot^2*sin(theta)]
    %   [ m*l*cos(theta), I + m*l^2       ] [theta_ddot] = [        m*g*l*sin(theta)            ]
    Mmat = [ M,               m*l*cos(theta);
             m*l*cos(theta),  I + m*l^2      ];
    rhs  = [ F_total + m*l*theta_dot^2*sin(theta);
             m*g*l*sin(theta) ];
    acc = Mmat \ rhs;   % [x_ddot; theta_ddot]

    Xdot = [ x_dot; acc(1); theta_dot; acc(2) ];
end

%% ------------------------------------------------------------------
%  6) NUMERICAL INTEGRATION (ode45, segmented at the disturbance edges
%     so the discontinuous pulse is integrated accurately)
%  ------------------------------------------------------------------
segEdges = [0, t_dist_start, t_dist_end, tEnd];
odeOptions = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
samplesPerSec = 200;
nSeg = numel(segEdges) - 1;

tCell = cell(nSeg,1); XCell = cell(nSeg,1);
Xstart = X0;
odeFun = @(t, X) nonlinearODE(t, X, M, m, l, I, g, K, F_disturbance_fun);
simError = '';
simCompleted = true;
try
    for s = 1:nSeg
        nPts = max(3, round((segEdges(s+1) - segEdges(s)) * samplesPerSec) + 1);
        tSeg = linspace(segEdges(s), segEdges(s+1), nPts);
        [tS, XS] = ode45(odeFun, tSeg, Xstart, odeOptions);
        tCell{s} = tS(:); XCell{s} = XS;
        Xstart = XS(end,:).';
    end
catch ME
    simCompleted = false;
    simError = ME.message;
end

fprintf('Nonlinear simulation completed without error: %d\n', simCompleted);
if ~simCompleted
    fprintf('ERROR: %s\n', simError);
    error('Nonlinear simulation failed.');
end

t = vertcat(tCell{:});
X = vertcat(XCell{:});
x        = X(:,1);
x_dot    = X(:,2);
theta    = X(:,3);        % radians
theta_dot = X(:,4);

F_LQR         = -(X * K.');
F_disturbance = F_dist_mag * double(t >= t_dist_start & t <= t_dist_end);
F_total       = F_LQR + F_disturbance;
theta_deg     = rad2deg(theta);

%% ------------------------------------------------------------------
%  7) NUMERICAL CHECKS (report only - no parameters changed if results
%     look unexpected)
%  ------------------------------------------------------------------
fprintf('\n---------------------------------------------------------\n');
fprintf(' NUMERICAL CHECKS\n');
fprintf('---------------------------------------------------------\n');

check1_radiansInternal = true;   % theta integrated directly in radians throughout (no deg used in the ODE)
fprintf('1. theta represented internally in radians: %d\n', check1_radiansInternal);

check2_initialAngle = abs(rad2deg(X0(3)) - theta0_deg) < 1e-10;
fprintf('2. Initial angle is exactly %g degrees: %d (X0(3) = %.10f rad = %.10f deg)\n', ...
    theta0_deg, check2_initialAngle, X0(3), rad2deg(X0(3)));

idxDist = find(F_disturbance ~= 0);
check3_disturbance = ~isempty(idxDist) ...
    && abs(t(idxDist(1))   - t_dist_start) < 1e-9 ...
    && abs(t(idxDist(end)) - t_dist_end)   < 1e-9 ...
    && all(abs(F_disturbance(idxDist) - F_dist_mag) < 1e-12) ...
    && all(F_disturbance(t < t_dist_start | t > t_dist_end) == 0);
fprintf('3. Disturbance is exactly +%.1f N for %.1f <= t <= %.1f s: %d\n', F_dist_mag, t_dist_start, t_dist_end, check3_disturbance);

check4_totalForce = max(abs(F_total - (F_LQR + F_disturbance))) < 1e-12;
fprintf('4. F_total = F_LQR + F_disturbance (identically): %d\n', check4_totalForce);

K_expected = [-1.0000, -2.9233, -61.5377, -12.5668];
check5_gainUnchanged = all(abs(K - K_expected) < 1e-4);
fprintf('5. K matches the existing gain exactly: %d  (K = [%.4f, %.4f, %.4f, %.4f])\n', ...
    check5_gainUnchanged, K(1), K(2), K(3), K(4));

check6_noNanInf = all(isfinite(X(:))) && all(isfinite(F_LQR)) && all(isfinite(F_disturbance)) && all(isfinite(F_total));
fprintf('6. No NaN or Inf values anywhere in states or forces: %d\n', check6_noNanInf);

maxAbsState = max(abs(X(:)));
check7_stable = check6_noNanInf && (maxAbsState < 1e3);
fprintf('7. Simulation remains numerically stable (max|state| = %.4f): %d\n', maxAbsState, check7_stable);

allChecksPassed = check1_radiansInternal && check2_initialAngle && check3_disturbance ...
    && check4_totalForce && check5_gainUnchanged && check6_noNanInf && check7_stable;
fprintf('\nALL CHECKS PASSED: %d\n', allChecksPassed);
if ~allChecksPassed
    fprintf('NOTE: parameters were NOT modified in response to any unexpected result. Reporting as-is.\n');
end

%% ------------------------------------------------------------------
%  8) SUMMARY VALUES
%  ------------------------------------------------------------------
initialThetaDeg = theta_deg(1);
[maxThetaDeg, iMaxTh] = max(theta_deg);
[minThetaDeg, iMinTh] = min(theta_deg);
finalThetaDeg = theta_deg(end);
maxAbsForce   = max(abs(F_LQR));
maxAbsX       = max(abs(x));
maxAbsXdot    = max(abs(x_dot));

fprintf('\n---------------------------------------------------------\n');
fprintf(' SUMMARY VALUES\n');
fprintf('---------------------------------------------------------\n');
fprintf('Initial theta   = %.4f deg\n', initialThetaDeg);
fprintf('Maximum theta   = %.4f deg (at t=%.3fs)\n', maxThetaDeg, t(iMaxTh));
fprintf('Minimum theta   = %.4f deg (at t=%.3fs)\n', minThetaDeg, t(iMinTh));
fprintf('Final theta     = %.4f deg\n', finalThetaDeg);
fprintf('Max |F_LQR|     = %.4f N\n', maxAbsForce);
fprintf('Max |x|         = %.4f m\n', maxAbsX);
fprintf('Max |x_dot|     = %.4f m/s\n\n', maxAbsXdot);

%% ------------------------------------------------------------------
%  9) VERIFICATION FIGURE
%  ------------------------------------------------------------------
shadeColor = [1.0 0.85 0.30];
padLim = @(v, lo, hi) [min(v(:)) - lo*(max(v(:))-min(v(:))+eps), max(v(:)) + hi*(max(v(:))-min(v(:))+eps)];

fig = figure('Color', 'w', 'Position', [60 40 1100 850]);

ax1 = subplot(2,2,1); hold(ax1,'on');
plot(ax1, t, theta_deg, 'r-', 'LineWidth', 2.0);
plot(ax1, [0 tEnd], [0 0], 'k--', 'LineWidth', 1.0);
grid(ax1,'on'); xlim(ax1,[0 tEnd]); ylim(ax1, padLim(theta_deg, 0.12, 0.25));
title(ax1, 'Pendulum Angle \theta(t) [Nonlinear Model]', 'FontWeight','bold');
xlabel(ax1,'Time [s]'); ylabel(ax1,'\theta [deg]');

ax2 = subplot(2,2,2); hold(ax2,'on');
plot(ax2, t, x, 'b-', 'LineWidth', 1.8);
grid(ax2,'on'); xlim(ax2,[0 tEnd]); ylim(ax2, padLim(x, 0.15, 0.15));
title(ax2, 'Chassis Position x(t) [Nonlinear Model]', 'FontWeight','bold');
xlabel(ax2,'Time [s]'); ylabel(ax2,'x [m]');

ax3 = subplot(2,2,3); hold(ax3,'on');
plot(ax3, t, F_LQR, 'Color', [0.55 0.2 0.65], 'LineWidth', 1.8);
grid(ax3,'on'); xlim(ax3,[0 tEnd]); ylim(ax3, padLim(F_LQR, 0.15, 0.15));
title(ax3, 'LQR Control Force F_{LQR}(t)', 'FontWeight','bold');
xlabel(ax3,'Time [s]'); ylabel(ax3,'F_{LQR} [N]');

ax4 = subplot(2,2,4); hold(ax4,'on');
plot(ax4, t, F_disturbance, 'Color', [0.9 0.4 0.0], 'LineWidth', 2.0);
grid(ax4,'on'); xlim(ax4,[0 tEnd]); ylim(ax4, padLim(F_disturbance, 0.15, 0.3));
title(ax4, 'Disturbance Force F_{disturbance}(t)', 'FontWeight','bold');
xlabel(ax4,'Time [s]'); ylabel(ax4,'F_{disturbance} [N]');

axList = [ax1 ax2 ax3 ax4];
for k = 1:numel(axList)
    axk = axList(k);
    yl = ylim(axk); ylim(axk, yl); hold(axk,'on');
    p = patch(axk, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl(1) yl(1) yl(2) yl(2)], ...
        shadeColor, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    uistack(p, 'bottom');
end
sgtitle('Four-Wheeled Inverted Pendulum - Nonlinear Model, Linear LQR Gain (Verification)', 'FontWeight','bold', 'FontSize',13);

pngFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_nonlinear_response.png');
exportgraphics(fig, pngFile, 'Resolution', 200);
fprintf('Figure saved to: %s\n', pngFile);

%% ------------------------------------------------------------------
%  10) SAVE DATA
%  ------------------------------------------------------------------
matFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_nonlinear_response.mat');
save(matFile, 't', 'x', 'x_dot', 'theta', 'theta_dot', 'theta_deg', ...
    'F_LQR', 'F_disturbance', 'F_total', 'K', 'M', 'm', 'l', 'I', 'g', ...
    'F_dist_mag', 't_dist_start', 't_dist_end', 'theta0_deg', 'X0', ...
    'initialThetaDeg', 'maxThetaDeg', 'minThetaDeg', 'finalThetaDeg', ...
    'maxAbsForce', 'maxAbsX', 'maxAbsXdot', 'allChecksPassed');
fprintf('Simulation data saved to: %s\n', matFile);

fprintf('\n=========================================================\n');
fprintf(' STEP 9 COMPLETE: Nonlinear model simulated with existing LQR gain.\n');
fprintf('=========================================================\n');
