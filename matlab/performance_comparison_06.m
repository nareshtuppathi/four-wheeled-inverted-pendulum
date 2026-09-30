%% Four-Wheeled Inverted Pendulum - LQR Performance Comparison
% STEP 6 of the robotics portfolio project.
%
% Compares two closed-loop simulations of the SAME LQR-controlled plant:
%   CASE A - LQR balancing without disturbance
%   CASE B - LQR balancing with a 2 N horizontal disturbance, 2.0 <= t <= 2.2 s
%
% Plant (A, B), weights (Q, R), gain K and initial condition are exactly
% those of Steps 2-5. Nothing is redesigned. In both cases:
%       F_LQR   = -K*X
%       F_total = F_LQR + F_disturbance
%       X_dot   = A*X + B*F_total
% (Case A simply uses F_disturbance = 0 for all time.) No Simulink here.

clear; clc; close all;

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS, STATE SPACE, LQR GAIN (unchanged)
%  ------------------------------------------------------------------
M = 2.50;      % equivalent chassis/base mass [kg]
m = 0.50;      % pendulum mass [kg]
l = 0.375;     % pivot-to-CoM distance [m]
I = 0.015;     % pendulum moment of inertia about its own CoM [kg*m^2]
g = 9.81;      % gravitational acceleration [m/s^2]

D = M*(I + m*l^2) - (m*l)^2;
A = [ 0   1   0                       0;
      0   0   -(g*m^2*l^2)/D          0;
      0   0   0                       1;
      0   0   (M*g*m*l)/D             0 ];
B = [ 0;  (m*l^2 + I)/D;  0;  -(m*l)/D ];

Q = diag([1, 1, 100, 10]);
R = 1;
K = lqr(A, B, Q, R);
K_step3 = [-1.0000, -2.9233, -61.5377, -12.5668];   % gain reported in Step 3
gainUnchanged = all(abs(K - K_step3) < 1e-3);

A_cl = A - B*K;
closedLoopEig = eig(A_cl);

fprintf('=========================================================\n');
fprintf(' STEP 6: LQR PERFORMANCE COMPARISON (Case A vs Case B)\n');
fprintf('=========================================================\n');
fprintf('LQR gain K = [%.4f, %.4f, %.4f, %.4f]\n\n', K(1), K(2), K(3), K(4));

%% ------------------------------------------------------------------
%  2) SIMULATION SETTINGS
%  ------------------------------------------------------------------
F_dist_mag   = 2.0;    % disturbance magnitude [N] (Case B)
t_dist_start = 2.0;    % disturbance start [s]
t_dist_end   = 2.2;    % disturbance end [s]
tEnd         = 5.0;    % simulation duration [s]
tolDeg       = 0.2;    % recovery tolerance band: |theta| <= tolDeg [deg]

theta0_deg = 5;                          % initial pendulum tilt [deg]
X0 = [0; 0; deg2rad(theta0_deg); 0];     % [x; x_dot; theta; theta_dot]

caseAmp   = [0, F_dist_mag];             % disturbance amplitude: Case A, Case B [N]
caseNames = {'No disturbance', 'With disturbance'};
nCases    = numel(caseAmp);

% Rectangular disturbance F_disturbance(t) = amp for t_dist_start <= t <= t_dist_end
F_disturbance = @(t, amp) amp * double(t >= t_dist_start & t <= t_dist_end);

%% ------------------------------------------------------------------
%  3) SIMULATE BOTH CASES WITH ode45
%  ------------------------------------------------------------------
% Both cases use the identical three-segment scheme (split at 2.0 s and
% 2.2 s) so the discontinuous pulse is integrated accurately and both runs
% share exactly the same time grid.
segEdges = [0, t_dist_start, t_dist_end, tEnd];
segMid   = (segEdges(1:end-1) + segEdges(2:end)) / 2;
nSeg     = numel(segMid);
samplesPerSec = 200;
odeOptions = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

tCase = cell(nCases, 1); XCase = cell(nCases, 1);
FdCase = cell(nCases, 1); segCase = cell(nCases, 1);
for c = 1:nCases
    segForce = F_disturbance(segMid, caseAmp(c));
    tCell = cell(nSeg, 1); XCell = cell(nSeg, 1);
    FdCell = cell(nSeg, 1); segCell = cell(nSeg, 1);
    Xstart = X0;
    for s = 1:nSeg
        nPts = max(3, round((segEdges(s+1) - segEdges(s)) * samplesPerSec) + 1);
        tSeg = linspace(segEdges(s), segEdges(s+1), nPts);
        Fd = segForce(s);
        odeFun = @(~, X) A*X + B*(-K*X + Fd);
        [tS, XS] = ode45(odeFun, tSeg, Xstart, odeOptions);
        tCell{s}   = tS(:);
        XCell{s}   = XS;
        FdCell{s}  = Fd * ones(numel(tS), 1);
        segCell{s} = s  * ones(numel(tS), 1);
        Xstart = XS(end, :).';
    end
    tCase{c}   = vertcat(tCell{:});
    XCase{c}   = vertcat(XCell{:});
    FdCase{c}  = vertcat(FdCell{:});
    segCase{c} = vertcat(segCell{:});
end

assert(isequal(tCase{1}, tCase{2}), 'Both cases must share the same time grid.');
tSim  = tCase{1};
segId = segCase{1};   % 1 = before pulse, 2 = pulse window, 3 = after pulse

% Assemble N-by-2 arrays: column 1 = Case A, column 2 = Case B
thetaDegAll = [rad2deg(XCase{1}(:,3)), rad2deg(XCase{2}(:,3))];   % pendulum angle [deg]
xAll        = [XCase{1}(:,1), XCase{2}(:,1)];                     % chassis position [m]
xdotAll     = [XCase{1}(:,2), XCase{2}(:,2)];                     % chassis velocity [m/s]
FdAll       = [FdCase{1}, FdCase{2}];                             % applied disturbance [N]
FlqrAll     = [-(XCase{1}*K.'), -(XCase{2}*K.')];                 % LQR force [N]
FtotAll     = FlqrAll + FdAll;                                    % total force [N]

%% ------------------------------------------------------------------
%  4) PERFORMANCE METRICS (both cases)
%  ------------------------------------------------------------------
metricVals = zeros(6, nCases);
for c = 1:nCases
    metricVals(1, c) = max(abs(thetaDegAll(:, c)));   % max |pendulum angle| [deg]
    metricVals(2, c) = thetaDegAll(end, c);           % final pendulum angle [deg]
    metricVals(3, c) = max(abs(xAll(:, c)));          % max |chassis position| [m]
    metricVals(4, c) = xAll(end, c);                  % final chassis position [m]
    metricVals(5, c) = max(abs(xdotAll(:, c)));       % max |chassis velocity| [m/s]
    metricVals(6, c) = max(abs(FlqrAll(:, c)));       % max |LQR control force| [N]
end

%% ------------------------------------------------------------------
%  5) CASE B EXTRA METRICS
%  ------------------------------------------------------------------
% Metric 7 - maximum angle deviation CAUSED by the disturbance:
%   the largest |theta_B(t) - theta_A(t)| for t >= 2.0 s. Because both cases
%   share the same initial condition, controller and time grid, this
%   difference isolates the effect of the pulse alone.
dTheta  = thetaDegAll(:, 2) - thetaDegAll(:, 1);
devMask = tSim >= t_dist_start;
tDevAll = tSim(devMask);
[devMax, iDev] = max(abs(dTheta(devMask)));
tDevMax = tDevAll(iDev);

% Metric 8 - recovery time after the disturbance:
%   the time after t = 2.2 s at which |theta_B| enters the band
%   |theta| <= tolerance and REMAINS inside it until the end of the run.
%   It is measured from t = 2.2 s (end of the pulse). The crossing time is
%   linearly interpolated between the last out-of-band sample and the next
%   one. If |theta_B| is still outside the band at the final sample, no
%   recovery time is reported (recovered = false, time = NaN).
thetaB    = thetaDegAll(:, 2);
postIdx   = find(segId == 3);
allTols   = [tolDeg, 0.5, 1.0];      % first entry = requested band; others are supplementary
recTime   = nan(size(allTols));
recovered = false(size(allTols));
for k = 1:numel(allTols)
    outIdx = postIdx(abs(thetaB(postIdx)) > allTols(k));
    if isempty(outIdx)
        recovered(k) = true;  recTime(k) = 0;
    elseif outIdx(end) < numel(tSim)
        j  = outIdx(end);
        a1 = abs(thetaB(j));  a2 = abs(thetaB(j+1));
        tCross = tSim(j) + (a1 - allTols(k)) / (a1 - a2) * (tSim(j+1) - tSim(j));
        recovered(k) = true;  recTime(k) = tCross - t_dist_end;
    end
end

%% ------------------------------------------------------------------
%  6) PERFORMANCE TABLE
%  ------------------------------------------------------------------
f4 = @(v) sprintf('%.4f', v);
metricNames = { ...
    'Max |pendulum angle| [deg]'; ...
    'Final pendulum angle [deg]'; ...
    'Max |chassis position| [m]'; ...
    'Final chassis position [m]'; ...
    'Max |chassis velocity| [m/s]'; ...
    'Max |LQR control force| [N]'; ...
    'Max angle deviation caused by disturbance [deg]'; ...
    sprintf('Recovery time after 2.2 s, |theta| <= %.1f deg [s]', tolDeg) };

colA = cell(8, 1);  colB = cell(8, 1);
for i = 1:6
    colA{i} = f4(metricVals(i, 1));
    colB{i} = f4(metricVals(i, 2));
end
colA{7} = 'n/a';  colB{7} = f4(devMax);
colA{8} = 'n/a';
if recovered(1)
    colB{8} = sprintf('%.3f', recTime(1));
else
    colB{8} = sprintf('not reached by %.1f s', tEnd);
end

try
    T = table(string(metricNames), string(colA), string(colB), ...
        'VariableNames', {'Metric', 'No Disturbance', 'With Disturbance'});
catch
    T = table(string(metricNames), string(colA), string(colB), ...
        'VariableNames', {'Metric', 'NoDisturbance', 'WithDisturbance'});
end

fprintf('---------------------------------------------------------\n');
fprintf(' PERFORMANCE COMPARISON TABLE\n');
fprintf('---------------------------------------------------------\n');
disp(T);

fprintf('---------------------------------------------------------\n');
fprintf(' RECOVERY TIME AND DEVIATION DETAILS (Case B)\n');
fprintf('---------------------------------------------------------\n');
fprintf('Max angle deviation caused by disturbance: %.4f deg (at t = %.3f s)\n', devMax, tDevMax);
fprintf('Recovery time, requested band |theta| <= %.1f deg (measured from t = %.1f s):\n', tolDeg, t_dist_end);
if recovered(1)
    fprintf('   %.3f s  (angle stays inside the band from t = %.3f s onward)\n', recTime(1), t_dist_end + recTime(1));
else
    fprintf('   NOT RECOVERED: |theta| is still above %.1f deg at t = %.1f s (final |theta| = %.4f deg).\n', ...
        tolDeg, tEnd, abs(thetaB(end)));
    fprintf('   No recovery time is reported for this band.\n');
end
fprintf('Supplementary recovery times for wider bands (Case B, informational):\n');
for k = 2:numel(allTols)
    if recovered(k)
        fprintf('   |theta| <= %.1f deg : %.3f s after the pulse ends\n', allTols(k), recTime(k));
    else
        fprintf('   |theta| <= %.1f deg : not reached by %.1f s\n', allTols(k), tEnd);
    end
end
fprintf('\n');

%% ------------------------------------------------------------------
%  7) SANITY CHECKS
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' SANITY CHECKS\n');
fprintf('---------------------------------------------------------\n');

% Check 1: both simulations completed over the full 0-5 s span.
simsCompleted = all(abs([tCase{1}(end), tCase{2}(end)] - tEnd) < 1e-9) ...
    && size(XCase{1}, 1) == size(XCase{2}, 1) && size(XCase{1}, 1) > 100;
fprintf('Check 1 (both simulations completed, t = 0..%.1f s):       %s\n', tEnd, mat2str(simsCompleted));

% Check 2: no NaN or Inf in any state or force.
allFinite = all(isfinite(XCase{1}(:))) && all(isfinite(XCase{2}(:))) ...
    && all(isfinite(FlqrAll(:))) && all(isfinite(FtotAll(:)));
fprintf('Check 2 (no NaN/Inf in states or forces):                 %s\n', mat2str(allFinite));

% Check 3: numerically stable - bounded states, closed-loop poles in the LHP.
maxAbsState = max([max(abs(XCase{1}(:))), max(abs(XCase{2}(:)))]);
closedLoopStable = all(real(closedLoopEig) < 0);
numericallyStable = allFinite && closedLoopStable && (maxAbsState < 1e3);
fprintf('Check 3 (both numerically stable, max|state| = %.3f):     %s\n', maxAbsState, mat2str(numericallyStable));

% Check 4: the disturbance acts only during 2.0-2.2 s (Case B), is absent in
% Case A, and Case A/B trajectories are identical before the pulse.
idxB = find(FdAll(:, 2) ~= 0);
distWindowOK = ~isempty(idxB) ...
    && abs(tSim(idxB(1))   - t_dist_start) < 1e-9 ...
    && abs(tSim(idxB(end)) - t_dist_end)   < 1e-9 ...
    && all(abs(FdAll(idxB, 2) - F_dist_mag) < 1e-12) ...
    && all(FdAll(segId ~= 2, 2) == 0) && all(FdAll(:, 1) == 0);
beforePulseIdentical = max(abs(XCase{1}(segId == 1, :) - XCase{2}(segId == 1, :)), [], 'all') < 1e-12;
fprintf('Check 4 (disturbance only in 2.0-2.2 s; runs match before): %s\n', mat2str(distWindowOK && beforePulseIdentical));

% Check 5: the disturbed system moves back toward upright after the pulse.
% Final |theta| must be below the post-pulse peak, and the mean |theta| in
% the last 0.5 s below the mean in the first 0.5 s after the pulse.
maxAbsAfterB  = max(abs(thetaB(segId == 3)));
earlyPostMask = (tSim > t_dist_end) & (tSim <= t_dist_end + 0.5);
lateMask      = (tSim >= tEnd - 0.5);
meanEarlyPost = mean(abs(thetaB(earlyPostMask)));
meanLate      = mean(abs(thetaB(lateMask)));
returnsUpright = (abs(thetaB(end)) < maxAbsAfterB) && (meanLate < meanEarlyPost);
fprintf('Check 5 (disturbed system recovers toward upright):        %s\n', mat2str(returnsUpright));
fprintf('         mean|theta| first 0.5 s after pulse = %.4f deg, last 0.5 s = %.4f deg\n', meanEarlyPost, meanLate);

% Check 6: K unchanged from Step 3.
fprintf('Check 6 (K unchanged from Step 3):                         %s\n', mat2str(gainUnchanged));

allChecksPassed = simsCompleted && allFinite && numericallyStable ...
    && distWindowOK && beforePulseIdentical && returnsUpright && gainUnchanged;
fprintf('\n');
if allChecksPassed
    fprintf('ALL SANITY CHECKS PASSED.\n\n');
else
    fprintf('WARNING: One or more sanity checks failed. Review the results.\n\n');
end

%% ------------------------------------------------------------------
%  8) FIGURE: FOUR-PANEL COMPARISON
%  ------------------------------------------------------------------
colA_rgb = [0.10 0.35 0.75];    % Case A: blue, solid
colB_rgb = [0.90 0.40 0.05];    % Case B: orange, dashed (drawn on top)
shadeColor = [1.0 0.85 0.30];
shadeName  = 'Disturbance: 2 N, 2.0-2.2 s';
padLim = @(v, lo, hi) [min(v(:)) - lo*(max(v(:))-min(v(:))), max(v(:)) + hi*(max(v(:))-min(v(:)))];

fig = figure('Color', 'w', 'Position', [60 40 1200 900]);

% --- Plot 1: pendulum angle comparison ---
ax1 = subplot(2,2,1); hold(ax1, 'on');
plot(ax1, tSim, thetaDegAll(:,1), '-',  'Color', colA_rgb, 'LineWidth', 2.2, 'DisplayName', 'No disturbance (Case A)');
plot(ax1, tSim, thetaDegAll(:,2), '--', 'Color', colB_rgb, 'LineWidth', 2.2, 'DisplayName', 'With 2 N disturbance (Case B)');
plot(ax1, [0 tEnd], [0 0], 'k-', 'LineWidth', 1.0, 'DisplayName', 'Upright (0 deg)');
plot(ax1, [0 tEnd], [tolDeg tolDeg], ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2, ...
    'DisplayName', sprintf('\\pm%.1f deg tolerance band', tolDeg));
plot(ax1, [0 tEnd], [-tolDeg -tolDeg], ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2, 'HandleVisibility', 'off');
grid(ax1, 'on'); xlim(ax1, [0 tEnd]); ylim(ax1, padLim(thetaDegAll, 0.12, 0.30));
title(ax1, 'Pendulum Angle \theta(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax1, 'Time [s]'); ylabel(ax1, '\theta [deg]');
if recovered(1)
    recText = sprintf('Case B recovery (|\\theta| <= %.1f^\\circ): %.2f s after pulse', tolDeg, recTime(1));
else
    recText = sprintf('Case B: |\\theta| <= %.1f^\\circ not reached by t = %.0f s (final %.3f^\\circ)', tolDeg, tEnd, abs(thetaB(end)));
end
text(ax1, 0.97, 0.50, recText, 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
    'FontSize', 8.5, 'BackgroundColor', 'w', 'EdgeColor', [0.7 0.7 0.7], 'Margin', 3);

% --- Plot 2: chassis position comparison ---
ax2 = subplot(2,2,2); hold(ax2, 'on');
plot(ax2, tSim, xAll(:,1), '-',  'Color', colA_rgb, 'LineWidth', 2.0, 'DisplayName', 'No disturbance (Case A)');
plot(ax2, tSim, xAll(:,2), '--', 'Color', colB_rgb, 'LineWidth', 2.0, 'DisplayName', 'With 2 N disturbance (Case B)');
grid(ax2, 'on'); xlim(ax2, [0 tEnd]); ylim(ax2, padLim(xAll, 0.12, 0.25));
title(ax2, 'Chassis Position x(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax2, 'Time [s]'); ylabel(ax2, 'x [m]');

% --- Plot 3: LQR control force comparison ---
ax3 = subplot(2,2,3); hold(ax3, 'on');
plot(ax3, tSim, FlqrAll(:,1), '-',  'Color', colA_rgb, 'LineWidth', 2.0, 'DisplayName', 'No disturbance (Case A)');
plot(ax3, tSim, FlqrAll(:,2), '--', 'Color', colB_rgb, 'LineWidth', 2.0, 'DisplayName', 'With 2 N disturbance (Case B)');
grid(ax3, 'on'); xlim(ax3, [0 tEnd]); ylim(ax3, padLim(FlqrAll, 0.12, 0.25));
title(ax3, 'LQR Control Force F_{LQR}(t) = -K X(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax3, 'Time [s]'); ylabel(ax3, 'F_{LQR} [N]');

% --- Plot 4: chassis velocity comparison ---
ax4 = subplot(2,2,4); hold(ax4, 'on');
plot(ax4, tSim, xdotAll(:,1), '-',  'Color', colA_rgb, 'LineWidth', 2.0, 'DisplayName', 'No disturbance (Case A)');
plot(ax4, tSim, xdotAll(:,2), '--', 'Color', colB_rgb, 'LineWidth', 2.0, 'DisplayName', 'With 2 N disturbance (Case B)');
grid(ax4, 'on'); xlim(ax4, [0 tEnd]); ylim(ax4, padLim(xdotAll, 0.12, 0.25));
title(ax4, 'Chassis Velocity x_{dot}(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax4, 'Time [s]'); ylabel(ax4, 'x_{dot} [m/s]');

% Shade the disturbance interval (2.0-2.2 s) on every plot.
axList = [ax1, ax2, ax3, ax4];
for k = 1:numel(axList)
    axk = axList(k);
    yl = ylim(axk);
    ylim(axk, yl);
    hold(axk, 'on');
    p = patch(axk, [t_dist_start t_dist_end t_dist_end t_dist_start], ...
        [yl(1) yl(1) yl(2) yl(2)], shadeColor, 'FaceAlpha', 0.35, ...
        'EdgeColor', 'none', 'DisplayName', shadeName);
    uistack(p, 'bottom');
    legend(axk, 'Location', 'best', 'FontSize', 8);
end

sgtitle('Four-Wheeled Inverted Pendulum - LQR Performance Comparison (5 deg initial tilt)', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ------------------------------------------------------------------
%% ------------------------------------------------------------------
%  9) SAVE FIGURE, DATA AND METRICS TABLE
%  ------------------------------------------------------------------

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Save Step 6 results in the organized results folder
resultsDir = fullfile(projectRoot, 'results', '06_performance_comparison');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

figFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_performance_comparison.png');

exportgraphics(fig, figFile, 'Resolution', 200);
fprintf('Figure saved to:        %s\n', figFile);

csvFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_performance_metrics.csv');

writetable(T, csvFile);
fprintf('Metrics table saved to: %s\n', csvFile);

matFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_performance_comparison.mat');

save(matFile, 'tSim', 'segId', 'thetaDegAll', 'xAll', 'xdotAll', 'FlqrAll', 'FdAll', 'FtotAll', ...
    'metricVals', 'devMax', 'tDevMax', 'allTols', 'recTime', 'recovered', 'T', ...
    'K', 'A', 'B', 'A_cl', 'Q', 'R', 'M', 'm', 'l', 'I', 'g', ...
    'F_dist_mag', 't_dist_start', 't_dist_end', 'tolDeg', 'theta0_deg', 'caseNames');

fprintf('Simulation data saved to: %s\n', matFile);