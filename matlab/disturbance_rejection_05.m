%% Four-Wheeled Inverted Pendulum - External Disturbance Rejection Test
% STEP 5 of the robotics portfolio project.
%
% Pipeline implemented in this script:
%   INITIAL 5 DEG TILT -> LQR BALANCING -> 2 N DISTURBANCE PULSE (2.0-2.2 s)
%   -> DISTURBANCE RESPONSE -> RECOVERY TOWARD UPRIGHT -> PLOTS AND METRICS
%
% The plant (A, B), the LQR weights (Q, R) and the LQR gain K are exactly
% those of Steps 2 and 3 - nothing is redesigned here. A short horizontal
% force pulse is added to the chassis on top of the LQR force:
%       F_LQR   = -K*X
%       F_total = F_LQR + F_disturbance
%       X_dot   = A*X + B*F_total
% No Simulink model and no animation are included in this step.

clear; clc; close all;

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS (unchanged from Steps 2-4)
%  ------------------------------------------------------------------
M = 2.50;      % equivalent chassis/base mass [kg]
m = 0.50;      % pendulum mass [kg]
l = 0.375;     % pivot-to-CoM distance [m]
I = 0.015;     % pendulum moment of inertia about its own CoM [kg*m^2]
g = 9.81;      % gravitational acceleration [m/s^2]

%% ------------------------------------------------------------------
%  2) LINEARIZED STATE-SPACE MODEL (unchanged from Step 2)
%     State vector: X = [x; x_dot; theta; theta_dot]
%     Input: F_total - total horizontal force on the chassis [N]
%  ------------------------------------------------------------------
D = M*(I + m*l^2) - (m*l)^2;

A = [ 0                       1   0                              0;
      0                       0   -(g*m^2*l^2)/D                 0;
      0                       0   0                              1;
      0                       0   (M*g*m*l)/D                    0 ];

B = [ 0;
      (m*l^2 + I)/D;
      0;
      -(m*l)/D ];

%% ------------------------------------------------------------------
%  3) LQR GAIN K (unchanged from Step 3: same Q and R)
%  ------------------------------------------------------------------
Q = diag([1, 1, 100, 10]);
R = 1;
K = lqr(A, B, Q, R);

% Consistency check: K must match the gain reported in Step 3.
K_step3 = [-1.0000, -2.9233, -61.5377, -12.5668];
gainUnchanged = all(abs(K - K_step3) < 1e-3);

A_cl = A - B*K;              % closed-loop matrix (without disturbance)
closedLoopEig = eig(A_cl);

fprintf('=========================================================\n');
fprintf(' STEP 5: EXTERNAL DISTURBANCE REJECTION\n');
fprintf('=========================================================\n');
fprintf('LQR gain K = [%.4f, %.4f, %.4f, %.4f]\n\n', K(1), K(2), K(3), K(4));

%% ------------------------------------------------------------------
%  4) DISTURBANCE DEFINITION
%     F_disturbance(t) = 2 N for 2.0 <= t <= 2.2 s, and 0 N otherwise
%  ------------------------------------------------------------------
F_dist_mag   = 2.0;   % disturbance force magnitude [N]
t_dist_start = 2.0;   % disturbance start time [s]
t_dist_end   = 2.2;   % disturbance end time [s]
tEnd         = 5.0;   % total simulation time [s]

F_disturbance = @(t) F_dist_mag * double(t >= t_dist_start & t <= t_dist_end);

%% ------------------------------------------------------------------
%  5) INITIAL CONDITIONS (same as Step 4)
%  ------------------------------------------------------------------
theta0_deg = 5;                          % initial pendulum tilt [deg]
X0 = [0; 0; deg2rad(theta0_deg); 0];     % [x; x_dot; theta; theta_dot]

fprintf('Initial conditions: x=0 m, x_dot=0 m/s, theta=%.1f deg, theta_dot=0 rad/s\n', theta0_deg);
fprintf('Disturbance: %.1f N from t=%.1f s to t=%.1f s\n\n', F_dist_mag, t_dist_start, t_dist_end);

%% ------------------------------------------------------------------
%  6) SIMULATION WITH ode45 (piecewise, split at the disturbance edges)
%  ------------------------------------------------------------------
% The disturbance is a discontinuous (rectangular) input. An adaptive
% solver can step across a discontinuity and lose accuracy, so the run is
% split into three segments whose boundaries coincide with the pulse edges:
%   segment 1: 0.0 -> 2.0 s (no disturbance)
%   segment 2: 2.0 -> 2.2 s (disturbance ON)
%   segment 3: 2.2 -> 5.0 s (no disturbance)
% The force is constant within each segment, and the state at the end of
% one segment is the initial condition of the next.
segEdges = [0, t_dist_start, t_dist_end, tEnd];
segMid   = (segEdges(1:end-1) + segEdges(2:end)) / 2;
segForce = F_disturbance(segMid);        % [0, 2, 0] N

samplesPerSec = 200;
odeOptions = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
nSeg = numel(segForce);

tCell = cell(nSeg, 1); XCell = cell(nSeg, 1);
FdCell = cell(nSeg, 1); segIdCell = cell(nSeg, 1);
Xstart = X0;
for s = 1:nSeg
    nPts = max(3, round((segEdges(s+1) - segEdges(s)) * samplesPerSec) + 1);
    tSeg = linspace(segEdges(s), segEdges(s+1), nPts);
    Fd = segForce(s);
    % Closed loop with disturbance: X_dot = A*X + B*(-K*X + F_disturbance)
    odeFun = @(~, X) A*X + B*(-K*X + Fd);
    [tS, XS] = ode45(odeFun, tSeg, Xstart, odeOptions);
    tCell{s}     = tS(:);
    XCell{s}     = XS;
    FdCell{s}    = Fd * ones(numel(tS), 1);
    segIdCell{s} = s  * ones(numel(tS), 1);
    Xstart = XS(end, :).';
end

tSim   = vertcat(tCell{:});
XSim   = vertcat(XCell{:});
F_dist = vertcat(FdCell{:});      % disturbance force actually applied [N]
segId  = vertcat(segIdCell{:});   % 1 = before, 2 = during, 3 = after

%% ------------------------------------------------------------------
%  7) FORCES
%  ------------------------------------------------------------------
F_LQR   = -(XSim * K.');           % LQR feedback force [N]
F_total = F_LQR + F_dist;          % total force applied to the chassis [N]

%% ------------------------------------------------------------------
%  8) STATE HISTORIES
%  ------------------------------------------------------------------
xHist    = XSim(:, 1);             % chassis position [m]
xDotHist = XSim(:, 2);             % chassis velocity [m/s]
thetaDeg = rad2deg(XSim(:, 3));    % pendulum angle [deg]

%% ------------------------------------------------------------------
%  9) PERFORMANCE METRICS
%  ------------------------------------------------------------------
idxBefore   = numel(tCell{1});     % last sample of segment 1 (t = 2.0 s, before pulse)
duringMask  = (segId == 2);
afterMask   = (segId == 3);

initialAngleDeg   = thetaDeg(1);
angleBeforeDistDeg = thetaDeg(idxBefore);
maxAbsAngleDuring = max(abs(thetaDeg(duringMask)));

thetaAfter = thetaDeg(afterMask);
tAfter     = tSim(afterMask);
[maxAbsAngleAfter, iPk] = max(abs(thetaAfter));
tPeakAfter         = tAfter(iPk);
thetaPeakAfterDeg  = thetaAfter(iPk);

finalAngleDeg = thetaDeg(end);
maxAbsF_LQR   = max(abs(F_LQR));
maxAbsF_total = max(abs(F_total));
finalPosition = xHist(end);
finalVelocity = xDotHist(end);

fprintf('---------------------------------------------------------\n');
fprintf(' PERFORMANCE METRICS\n');
fprintf('---------------------------------------------------------\n');
fprintf('Initial pendulum angle                     : %9.4f deg\n', initialAngleDeg);
fprintf('Pendulum angle just before disturbance     : %9.4f deg  (t = %.1f s)\n', angleBeforeDistDeg, t_dist_start);
fprintf('Max |angle| during disturbance (2.0-2.2 s) : %9.4f deg\n', maxAbsAngleDuring);
fprintf('Max |angle| after disturbance (t > 2.2 s)  : %9.4f deg  (at t = %.3f s)\n', maxAbsAngleAfter, tPeakAfter);
fprintf('Final pendulum angle                       : %9.4f deg\n', finalAngleDeg);
fprintf('Max |F_LQR|                                : %9.4f N\n', maxAbsF_LQR);
fprintf('Max |F_total|                              : %9.4f N\n', maxAbsF_total);
fprintf('Final chassis position                     : %9.4f m\n', finalPosition);
fprintf('Final chassis velocity                     : %9.4f m/s\n\n', finalVelocity);

%% ------------------------------------------------------------------
%  10) AUTOMATIC CHECKS
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' AUTOMATIC CHECKS\n');
fprintf('---------------------------------------------------------\n');

% Check 1: no NaN or Inf anywhere in states or forces.
allFinite = all(isfinite(XSim(:))) && all(isfinite(F_LQR)) && all(isfinite(F_total));
fprintf('Check 1 (no NaN/Inf in states or forces):                 %s\n', mat2str(allFinite));

% Check 2: the disturbance is applied exactly in [2.0, 2.2] s at 2 N and
% is zero at every other sample.
appliedIdx = find(F_dist ~= 0);
distWindowOK = ~isempty(appliedIdx) ...
    && abs(tSim(appliedIdx(1))   - t_dist_start) < 1e-9 ...
    && abs(tSim(appliedIdx(end)) - t_dist_end)   < 1e-9 ...
    && all(abs(F_dist(appliedIdx) - F_dist_mag) < 1e-12) ...
    && all(F_dist(segId ~= 2) == 0);
fprintf('Check 2 (2 N applied only for 2.0 <= t <= 2.2 s):         %s\n', mat2str(distWindowOK));

% Check 3: numerically stable - finite, bounded states, and all
% closed-loop eigenvalues in the left half-plane.
maxAbsState = max(abs(XSim(:)));
closedLoopStable = all(real(closedLoopEig) < 0);
numericallyStable = allFinite && closedLoopStable && (maxAbsState < 1e3);
fprintf('Check 3 (numerically stable, max|state| = %.3f):        %s\n', maxAbsState, mat2str(numericallyStable));

% Check 4: the pendulum returns toward upright after the disturbance.
% Compares the mean |angle| in the first 0.5 s after the pulse with the
% mean |angle| in the last 0.5 s of the run, and requires the final
% |angle| to be below the post-disturbance peak.
earlyPostMask = (tSim > t_dist_end) & (tSim <= t_dist_end + 0.5);
lateMask      = (tSim >= tEnd - 0.5);
meanAbsEarlyPost = mean(abs(thetaDeg(earlyPostMask)));
meanAbsLate      = mean(abs(thetaDeg(lateMask)));
returnsUpright = (abs(finalAngleDeg) < maxAbsAngleAfter) && (meanAbsLate < meanAbsEarlyPost);
fprintf('Check 4 (angle returns toward upright after pulse):       %s\n', mat2str(returnsUpright));
fprintf('         mean|theta| first 0.5 s after pulse = %.4f deg, last 0.5 s = %.4f deg\n', meanAbsEarlyPost, meanAbsLate);

% Check 5: controller unchanged (K matches Step 3).
fprintf('Check 5 (K unchanged from Step 3):                        %s\n', mat2str(gainUnchanged));

% Informational only (not part of the pass/fail result): is the final
% angle inside a 5%% settling band of the initial 5 deg tilt?
settleBandDeg = 0.05 * theta0_deg;
fprintf('Info    (final |angle| within %.2f deg settling band):      %s\n', settleBandDeg, mat2str(abs(finalAngleDeg) < settleBandDeg));

allChecksPassed = allFinite && distWindowOK && numericallyStable && returnsUpright && gainUnchanged;
fprintf('\n');
if allChecksPassed
    fprintf('ALL AUTOMATIC CHECKS PASSED.\n\n');
else
    fprintf('WARNING: One or more checks failed. Review the simulation.\n\n');
end

%% ------------------------------------------------------------------
%  11) FIGURE: FOUR-PANEL DISTURBANCE RESPONSE
%  ------------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [60 40 1200 900]);
padLim = @(v, lo, hi) [min(v) - lo*(max(v)-min(v)), max(v) + hi*(max(v)-min(v))];
shadeColor = [1.0 0.85 0.30];
shadeName  = 'Disturbance: 2 N, 2.0-2.2 s';

% --- Plot 1: chassis position x(t) ---
ax1 = subplot(2,2,1);
plot(ax1, tSim, xHist, 'b-', 'LineWidth', 1.8, 'DisplayName', 'x(t)');
grid(ax1, 'on'); xlim(ax1, [0 tEnd]); ylim(ax1, padLim(xHist, 0.15, 0.15));
title(ax1, 'Chassis Position x(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax1, 'Time [s]'); ylabel(ax1, 'x [m]');

% --- Plot 2: chassis velocity x_dot(t) ---
ax2 = subplot(2,2,2);
plot(ax2, tSim, xDotHist, 'Color', [0.1 0.5 0.2], 'LineWidth', 1.8, 'DisplayName', 'x_{dot}(t)');
grid(ax2, 'on'); xlim(ax2, [0 tEnd]); ylim(ax2, padLim(xDotHist, 0.15, 0.15));
title(ax2, 'Chassis Velocity x_{dot}(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax2, 'Time [s]'); ylabel(ax2, 'x_{dot} [m/s]');

% --- Plot 3: pendulum angle theta(t) with annotated phases ---
ax3 = subplot(2,2,3);
hold(ax3, 'on');
plot(ax3, tSim, thetaDeg, 'r-', 'LineWidth', 2.0, 'DisplayName', '\theta(t)');
plot(ax3, [0 tEnd], [0 0], 'k--', 'LineWidth', 1.2, 'DisplayName', 'Upright (0 deg)');
plot(ax3, 0, thetaDeg(1), 'ko', 'MarkerFaceColor', [1 0.9 0], 'MarkerSize', 9, ...
    'DisplayName', ['Initial tilt (' num2str(theta0_deg) '^\circ)']);
plot(ax3, t_dist_start, angleBeforeDistDeg, 'ks', 'MarkerFaceColor', [0.4 0.9 0.9], 'MarkerSize', 8, ...
    'DisplayName', 'Just before disturbance');
plot(ax3, tPeakAfter, thetaPeakAfterDeg, 'kv', 'MarkerFaceColor', [0.9 0.5 0.9], 'MarkerSize', 8, ...
    'DisplayName', 'Peak after disturbance');
grid(ax3, 'on'); xlim(ax3, [0 tEnd]); ylim(ax3, padLim(thetaDeg, 0.15, 0.35));
title(ax3, 'Pendulum Angle \theta(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax3, 'Time [s]'); ylabel(ax3, '\theta [deg]');

% --- Plot 4: forces ---
ax4 = subplot(2,2,4);
hold(ax4, 'on');
plot(ax4, tSim, F_LQR,   'Color', [0.55 0.2 0.65], 'LineWidth', 1.8, 'DisplayName', 'LQR force F_{LQR}');
plot(ax4, tSim, F_dist,  'Color', [0.9 0.4 0.0],  'LineWidth', 2.0, 'DisplayName', 'Disturbance F_{dist}');
plot(ax4, tSim, F_total, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Total F_{total}');
grid(ax4, 'on'); xlim(ax4, [0 tEnd]); ylim(ax4, padLim([F_LQR; F_dist; F_total], 0.15, 0.15));
title(ax4, 'Forces on the Chassis', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax4, 'Time [s]'); ylabel(ax4, 'Force [N]');

% Shade the disturbance interval (2.0-2.2 s) on every plot.
axList = [ax1, ax2, ax3, ax4];
for k = 1:numel(axList)
    axk = axList(k);
    yl = ylim(axk);
    ylim(axk, yl);   % freeze limits so the shading spans the full height
    hold(axk, 'on');
    p = patch(axk, [t_dist_start t_dist_end t_dist_end t_dist_start], ...
        [yl(1) yl(1) yl(2) yl(2)], shadeColor, 'FaceAlpha', 0.35, ...
        'EdgeColor', 'none', 'DisplayName', shadeName);
    uistack(p, 'bottom');
end

% Phase labels on the angle plot (positions in normalized axes units).
text(ax3, 1.0/tEnd, 0.94, 'Initial stabilization', 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
text(ax3, 2.1/tEnd, 0.94, 'Disturbance', 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9, 'Color', [0.8 0.4 0]);
text(ax3, 3.8/tEnd, 0.94, 'Recovery', 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);

legend(ax1, 'Location', 'best');
legend(ax2, 'Location', 'best');
legend(ax3, 'Location', 'best', 'FontSize', 8);
legend(ax4, 'Location', 'best');

sgtitle('Four-Wheeled Inverted Pendulum - LQR Disturbance Rejection (2 N pulse, 2.0-2.2 s)', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ------------------------------------------------------------------
%% ------------------------------------------------------------------
%  12) SAVE FIGURE AND DATA
%  ------------------------------------------------------------------

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Save Step 5 results in the organized results folder
resultsDir = fullfile(projectRoot, 'results', '05_disturbance_rejection');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

figFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_disturbance_response.png');

exportgraphics(fig, figFile, 'Resolution', 200);
fprintf('Figure saved to: %s\n', figFile);

matFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_disturbance_response.mat');

save(matFile, 'tSim', 'XSim', 'F_LQR', 'F_dist', 'F_total', 'segId', ...
    'K', 'A', 'B', 'A_cl', 'Q', 'R', 'M', 'm', 'l', 'I', 'g', ...
    'F_dist_mag', 't_dist_start', 't_dist_end', 'theta0_deg', ...
    'initialAngleDeg', 'angleBeforeDistDeg', 'maxAbsAngleDuring', ...
    'maxAbsAngleAfter', 'tPeakAfter', 'finalAngleDeg', 'maxAbsF_LQR', ...
    'maxAbsF_total', 'finalPosition', 'finalVelocity');

fprintf('Simulation data saved to: %s\n', matFile);