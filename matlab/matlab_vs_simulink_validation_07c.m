%% Four-Wheeled Inverted Pendulum - MATLAB vs Simulink Validation
% STEP 7C of the robotics portfolio project.
%
% Compares the SAME scenario (LQR + 2 N disturbance, 2.0-2.2 s) computed
% two independent ways:
%   1) MATLAB ode45, reproducing four_wheeled_inverted_pendulum_disturbance.m
%      (Step 5's segmented ode45 approach) with identical A, B, K, X0.
%   2) The Simulink model four_wheeled_inverted_pendulum_lqr_simulink.slx,
%      loaded from its already-saved output
%      four_wheeled_inverted_pendulum_simulink_response.mat (Step 7B).
% A, B, K, Q, R, X0, disturbance magnitude/timing and the Simulink model
% are NOT modified or re-tuned here.

clear; clc; close all;

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Step 7 Simulink results
resultsDir = fullfile(projectRoot, 'results', '07_simulink');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% ------------------------------------------------------------------
%  1) MATLAB ODE45 REPRODUCTION (same physics/method as Step 5)
%  ------------------------------------------------------------------
M = 2.50;  m = 0.50;  l = 0.375;  I = 0.015;  g = 9.81;
D = M*(I + m*l^2) - (m*l)^2;
A = [ 0 1 0 0; 0 0 -(g*m^2*l^2)/D 0; 0 0 0 1; 0 0 (M*g*m*l)/D 0 ];
B = [ 0; (m*l^2 + I)/D; 0; -(m*l)/D ];
Q = diag([1, 1, 100, 10]);  R = 1;
K = lqr(A, B, Q, R);

F_dist_mag   = 2.0;  t_dist_start = 2.0;  t_dist_end = 2.2;  tEnd = 5.0;
theta0_deg   = 5;    X0 = [0; 0; deg2rad(theta0_deg); 0];

fprintf('=========================================================\n');
fprintf(' STEP 7C: MATLAB vs SIMULINK VALIDATION\n');
fprintf('=========================================================\n');
fprintf('K = [%.4f, %.4f, %.4f, %.4f]\n\n', K(1), K(2), K(3), K(4));

% Same three-segment ode45 scheme as Step 5, split exactly at the pulse edges.
segEdges = [0, t_dist_start, t_dist_end, tEnd];
segMid   = (segEdges(1:end-1) + segEdges(2:end)) / 2;
segForce = F_dist_mag * double(segMid >= t_dist_start & segMid <= t_dist_end);
samplesPerSec = 200;
odeOptions = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
nSeg = numel(segForce);

tCell = cell(nSeg,1); XCell = cell(nSeg,1); FdCell = cell(nSeg,1);
Xstart = X0;
for s = 1:nSeg
    nPts = max(3, round((segEdges(s+1) - segEdges(s)) * samplesPerSec) + 1);
    tSeg = linspace(segEdges(s), segEdges(s+1), nPts);
    Fd = segForce(s);
    odeFun = @(~, X) A*X + B*(-K*X + Fd);
    [tS, XS] = ode45(odeFun, tSeg, Xstart, odeOptions);
    tCell{s} = tS(:); XCell{s} = XS; FdCell{s} = Fd*ones(numel(tS),1);
    Xstart = XS(end,:).';
end
t_mat      = vertcat(tCell{:});
X_mat      = vertcat(XCell{:});
Fdist_mat  = vertcat(FdCell{:});
FLQR_mat   = -(X_mat * K.');
Ftotal_mat = FLQR_mat + Fdist_mat;

x_mat     = X_mat(:,1);
xdot_mat  = X_mat(:,2);
theta_mat = rad2deg(X_mat(:,3));

fprintf('MATLAB ode45 run: %d samples over t = [%.3f, %.3f] s\n\n', numel(t_mat), t_mat(1), t_mat(end));

%% ------------------------------------------------------------------
%  2) LOAD THE SAVED SIMULINK RESPONSE
%  ------------------------------------------------------------------
simFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_simulink_response.mat');
assert(isfile(simFile), 'Simulink response file not found: %s', simFile);
S = load(simFile);

t_sl      = S.tSim(:);
x_sl      = S.xHist(:);
xdot_sl   = S.xdHist(:);
theta_sl  = S.thHist(:);
FLQR_sl   = S.FLQR(:);
Fdist_sl  = S.Fdist(:);
Ftotal_sl = S.Ftotal(:);

fprintf('Loaded Simulink response: %d samples over t = [%.3f, %.3f] s\n\n', numel(t_sl), t_sl(1), t_sl(end));

%% ------------------------------------------------------------------
%  3) INTERPOLATE ONTO A COMMON TIME VECTOR
%  ------------------------------------------------------------------
% The two runs use different adaptive solver grids. A common, evenly
% spaced 1 ms grid (5001 points over 0-5 s) is used so both are resampled
% by the SAME amount rather than interpolating one onto the other's grid,
% which would bias the comparison toward whichever grid is coarser.
tCommon = (0:0.001:tEnd).';

% Segment boundaries (MATLAB's piecewise ode45) and solver zero-crossing
% events (Simulink's Step blocks) both land exactly on t=2.0s and t=2.2s
% from both sides, producing duplicate time stamps at those instants.
% interp1 requires strictly increasing sample points, so duplicates are
% collapsed to their LAST occurrence (the post-transition value, i.e. the
% value immediately after the disturbance edge) before interpolating.
[t_mat_u, ia_mat] = unique(t_mat, 'last');
[t_sl_u,  ia_sl ]  = unique(t_sl,  'last');

% theta, x, xdot, F_LQR are smooth/continuous - linear interpolation is
% appropriate. F_dist and F_total are DISCONTINUOUS rectangular-pulse
% signals; linearly interpolating them onto a common grid draws a ramp
% across each edge where the two solvers' native sample times do not
% coincide, producing a large spurious difference at a handful of
% samples (see the Step 7C investigation). Zero-order-hold ('previous')
% interpolation reproduces the ideal step for both sources instead.
interpMatLin  = @(t, y) interp1(t, y, tCommon, 'linear',   'extrap');
interpMatStep = @(t, y) interp1(t, y, tCommon, 'previous', 'extrap');
theta_M = interpMatLin(t_mat_u, theta_mat(ia_mat));  theta_S = interpMatLin(t_sl_u, theta_sl(ia_sl));
x_M     = interpMatLin(t_mat_u, x_mat(ia_mat));      x_S     = interpMatLin(t_sl_u, x_sl(ia_sl));
xdot_M  = interpMatLin(t_mat_u, xdot_mat(ia_mat));   xdot_S  = interpMatLin(t_sl_u, xdot_sl(ia_sl));
FLQR_M  = interpMatLin(t_mat_u, FLQR_mat(ia_mat));   FLQR_S  = interpMatLin(t_sl_u, FLQR_sl(ia_sl));
Fdist_M = interpMatStep(t_mat_u, Fdist_mat(ia_mat));  Fdist_S = interpMatStep(t_sl_u, Fdist_sl(ia_sl));
Ftot_M  = interpMatStep(t_mat_u, Ftotal_mat(ia_mat)); Ftot_S  = interpMatStep(t_sl_u, Ftotal_sl(ia_sl));

fprintf('Common time grid: %d points, dt = %.4f s\n\n', numel(tCommon), tCommon(2)-tCommon(1));

%% ------------------------------------------------------------------
%  4)-5) COMPARISON METRICS (max abs diff and RMSE per signal)
%  ------------------------------------------------------------------
sigNames = {'theta [deg]', 'x [m]', 'x_dot [m/s]', 'F_LQR [N]', 'F_dist [N]', 'F_total [N]'};
Msig = {theta_M, x_M, xdot_M, FLQR_M, Fdist_M, Ftot_M};
Ssig = {theta_S, x_S, xdot_S, FLQR_S, Fdist_S, Ftot_S};

nSig = numel(sigNames);
maxAbsDiff = zeros(nSig,1); rmse = zeros(nSig,1); signalRange = zeros(nSig,1);
for k = 1:nSig
    d = Msig{k} - Ssig{k};
    maxAbsDiff(k) = max(abs(d));
    rmse(k) = sqrt(mean(d.^2));
    signalRange(k) = max(Msig{k}) - min(Msig{k});
end

fprintf('---------------------------------------------------------\n');
fprintf(' COMPARISON METRICS (MATLAB vs Simulink)\n');
fprintf('---------------------------------------------------------\n');
fprintf('%-14s %14s %14s\n', 'Signal', 'Max|diff|', 'RMSE');
for k = 1:nSig
    fprintf('%-14s %14.6f %14.6f\n', sigNames{k}, maxAbsDiff(k), rmse(k));
end
fprintf('\n');

%% ------------------------------------------------------------------
%  6) AGREEMENT DETERMINATION
%  ------------------------------------------------------------------
% Tolerance is defined relative to each signal's own MATLAB range (1% of
% range), rather than one fixed absolute number, since the six signals
% have very different units and magnitudes (degrees, meters, m/s, N).
tolFrac = 0.01;
tolAbs  = max(tolFrac * signalRange, 1e-6);
agreesPerSignal = maxAbsDiff <= tolAbs;

fprintf('---------------------------------------------------------\n');
fprintf(' AGREEMENT CHECK (tolerance = 1%% of each signal''s MATLAB range)\n');
fprintf('---------------------------------------------------------\n');
for k = 1:nSig
    fprintf('%-14s tol=%10.6f  max|diff|=%10.6f  agrees=%d\n', ...
        sigNames{k}, tolAbs(k), maxAbsDiff(k), agreesPerSignal(k));
end
overallAgree = all(agreesPerSignal);
fprintf('\nOverall agreement within tolerance: %d\n\n', overallAgree);

%% ------------------------------------------------------------------
%  7) COMPARISON FIGURE
%  ------------------------------------------------------------------
colM = [0.10 0.35 0.75];  colS = [0.90 0.40 0.05];
shadeColor = [1.0 0.85 0.30];
padLim = @(v, lo, hi) [min(v(:)) - lo*(max(v(:))-min(v(:))+eps), max(v(:)) + hi*(max(v(:))-min(v(:))+eps)];

fig = figure('Color', 'w', 'Position', [50 30 1200 900]);

ax1 = subplot(2,2,1); hold(ax1,'on');
plot(ax1, tCommon, theta_M, '-',  'Color', colM, 'LineWidth', 2.0, 'DisplayName', 'MATLAB (ode45)');
plot(ax1, tCommon, theta_S, '--', 'Color', colS, 'LineWidth', 2.0, 'DisplayName', 'Simulink');
grid(ax1,'on'); xlim(ax1,[0 tEnd]); ylim(ax1, padLim([theta_M; theta_S], 0.12, 0.25));
title(ax1, '(a) Pendulum Angle \theta(t)', 'FontWeight','bold'); xlabel(ax1,'Time [s]'); ylabel(ax1,'\theta [deg]');

ax2 = subplot(2,2,2); hold(ax2,'on');
plot(ax2, tCommon, x_M, '-',  'Color', colM, 'LineWidth', 2.0, 'DisplayName', 'MATLAB (ode45)');
plot(ax2, tCommon, x_S, '--', 'Color', colS, 'LineWidth', 2.0, 'DisplayName', 'Simulink');
grid(ax2,'on'); xlim(ax2,[0 tEnd]); ylim(ax2, padLim([x_M; x_S], 0.12, 0.25));
title(ax2, '(b) Chassis Position x(t)', 'FontWeight','bold'); xlabel(ax2,'Time [s]'); ylabel(ax2,'x [m]');

ax3 = subplot(2,2,3); hold(ax3,'on');
plot(ax3, tCommon, xdot_M, '-',  'Color', colM, 'LineWidth', 2.0, 'DisplayName', 'MATLAB (ode45)');
plot(ax3, tCommon, xdot_S, '--', 'Color', colS, 'LineWidth', 2.0, 'DisplayName', 'Simulink');
grid(ax3,'on'); xlim(ax3,[0 tEnd]); ylim(ax3, padLim([xdot_M; xdot_S], 0.12, 0.25));
title(ax3, '(c) Chassis Velocity x_{dot}(t)', 'FontWeight','bold'); xlabel(ax3,'Time [s]'); ylabel(ax3,'x_{dot} [m/s]');

ax4 = subplot(2,2,4); hold(ax4,'on');
plot(ax4, tCommon, FLQR_M, '-',  'Color', colM, 'LineWidth', 2.0, 'DisplayName', 'MATLAB (ode45)');
plot(ax4, tCommon, FLQR_S, '--', 'Color', colS, 'LineWidth', 2.0, 'DisplayName', 'Simulink');
grid(ax4,'on'); xlim(ax4,[0 tEnd]); ylim(ax4, padLim([FLQR_M; FLQR_S], 0.12, 0.25));
title(ax4, '(d) LQR Control Force F_{LQR}(t)', 'FontWeight','bold'); xlabel(ax4,'Time [s]'); ylabel(ax4,'F_{LQR} [N]');

axList = [ax1 ax2 ax3 ax4];
for k = 1:numel(axList)
    axk = axList(k);
    yl = ylim(axk); ylim(axk, yl); hold(axk,'on');
    p = patch(axk, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl(1) yl(1) yl(2) yl(2)], ...
        shadeColor, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'DisplayName', 'Disturbance 2.0-2.2 s');
    uistack(p, 'bottom');
    legend(axk, 'Location', 'best', 'FontSize', 8);
end

sgtitle('Four-Wheeled Inverted Pendulum - MATLAB vs Simulink Validation', 'FontWeight','bold', 'FontSize',14);

figFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_matlab_vs_simulink.png');
exportgraphics(fig, figFile, 'Resolution', 200);
fprintf('Figure saved to: %s\n', figFile);

%% ------------------------------------------------------------------
%  8)-9) SAVE COMPARISON TABLE (CSV) AND DATA (MAT)
%  ------------------------------------------------------------------
T = table(sigNames(:), maxAbsDiff, rmse, tolAbs, agreesPerSignal, ...
    'VariableNames', {'Signal', 'MaxAbsDiff', 'RMSE', 'ToleranceUsed', 'AgreesWithinTolerance'});
fprintf('---------------------------------------------------------\n');
fprintf(' COMPARISON TABLE\n');
fprintf('---------------------------------------------------------\n');
disp(T);

csvFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_matlab_vs_simulink.csv');
writetable(T, csvFile);
fprintf('Comparison table saved to: %s\n', csvFile);

matFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_matlab_vs_simulink.mat');
save(matFile, 'tCommon', 'theta_M','theta_S','x_M','x_S','xdot_M','xdot_S', ...
    'FLQR_M','FLQR_S','Fdist_M','Fdist_S','Ftot_M','Ftot_S', ...
    'maxAbsDiff','rmse','tolAbs','agreesPerSignal','overallAgree','sigNames', ...
    'A','B','K','Q','R','X0','F_dist_mag','t_dist_start','t_dist_end','tEnd');
fprintf('Comparison data saved to: %s\n', matFile);

fprintf('\n=========================================================\n');
fprintf(' STEP 7C COMPLETE: MATLAB vs Simulink comparison finished.\n');
fprintf(' Overall agreement within tolerance: %d\n', overallAgree);
fprintf('=========================================================\n');
