%% Run and validate the Simulink model four_wheeled_inverted_pendulum_lqr_simulink.slx
% STEP 7B of the robotics portfolio project.
%
% Runs the verified Simulink model exactly as built (A, B, K, Q, R,
% initial condition, and disturbance timing/magnitude are NOT modified
% here - they come from the model's own InitFcn, identical to Steps 2-6).
% This script only executes the simulation, inspects the logged signals,
% plots them, and saves the results. No MATLAB-vs-Simulink comparison yet.

clear; clc; close all;

mdl = 'four_wheeled_inverted_pendulum_lqr';

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Simulink model location
simulinkDir = fullfile(projectRoot, 'simulink');
slxFile = fullfile(simulinkDir, [mdl '.slx']);

% Step 7 results
resultsDir = fullfile(projectRoot, 'results', '07_simulink');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
fprintf('=========================================================\n');
fprintf(' STEP 7B: RUN AND VALIDATE THE SIMULINK MODEL\n');
fprintf('=========================================================\n');

%% ------------------------------------------------------------------
%  1) LOAD AND RUN THE MODEL
%  ------------------------------------------------------------------
if ~bdIsLoaded(mdl)
    load_system(slxFile);
end

% The model's InitFcn defines M, m, l, I, g, A, B, K, X0, theta0_deg,
% F_dist_mag, t_dist_start, t_dist_end, tEnd in the base workspace -
% exactly the same formulas as Steps 2-6. Nothing is redefined here.
simOut = [];
simError = '';
try
    simOut = sim(mdl);
    simCompleted = true;
catch ME
    simCompleted = false;
    simError = ME.message;
end

fprintf('Simulation completed without error: %d\n', simCompleted);
if ~simCompleted
    fprintf('ERROR: %s\n', simError);
    error('Simulation failed - aborting Step 7B.');
end

% NOTE: the model has ReturnWorkspaceOutputs = 'on', which routes all
% To Workspace / logged signals into the returned Simulink.SimulationOutput
% object (simOut) instead of writing them directly to the base workspace.
% This is standard Simulink behavior when the sim() output is captured in
% a variable, and is not a modification of the model - the signals are
% extracted from simOut below instead of the base workspace.
fprintf('ReturnWorkspaceOutputs setting: %s\n\n', get_param(mdl, 'ReturnWorkspaceOutputs'));

%% ------------------------------------------------------------------
%  2) EXTRACT LOGGED SIGNALS (from the SimulationOutput object)
%  ------------------------------------------------------------------
reqVars = {'x_sim','xdot_sim','theta_deg_sim','thetadot_sim', 'F_LQR_sim','F_dist_sim','F_total_sim'};
existMask = false(size(reqVars));
for k = 1:numel(reqVars)
    existMask(k) = isfield(get(simOut), reqVars{k}) || any(strcmp(simOut.who, reqVars{k}));
end
missing = reqVars(~existMask);
assert(isempty(missing), 'Missing logged variables in SimulationOutput: %s', strjoin(missing, ', '));

x_ts        = simOut.get('x_sim');
xdot_ts     = simOut.get('xdot_sim');
thetaDeg_ts = simOut.get('theta_deg_sim');
thetadot_ts = simOut.get('thetadot_sim');
FLQR_ts     = simOut.get('F_LQR_sim');
Fdist_ts    = simOut.get('F_dist_sim');
Ftotal_ts   = simOut.get('F_total_sim');

% All signals are logged on their own solver-driven time vectors; resample
% everything onto the finest common time base (the state signal's grid)
% using linear interpolation so the six series can be plotted and analyzed
% together.
tGrid = x_ts.Time;
resample = @(ts) interp1(ts.Time, ts.Data, tGrid, 'linear', 'extrap');

tSim     = tGrid;
xHist    = resample(x_ts);
xdHist   = resample(xdot_ts);
thHist   = resample(thetaDeg_ts);
thdHist  = resample(thetadot_ts);
FLQR     = resample(FLQR_ts);
Fdist    = resample(Fdist_ts);
Ftotal   = resample(Ftotal_ts);

fprintf('Logged samples: %d points over t = [%.3f, %.3f] s\n\n', numel(tSim), tSim(1), tSim(end));

%% ------------------------------------------------------------------
%  3) PARAMETERS USED (pulled from base workspace, set by InitFcn)
%  ------------------------------------------------------------------
theta0_deg   = evalin('base', 'theta0_deg');
t_dist_start = evalin('base', 't_dist_start');
t_dist_end   = evalin('base', 't_dist_end');
F_dist_mag   = evalin('base', 'F_dist_mag');
tEnd         = evalin('base', 'tEnd');

%% ------------------------------------------------------------------
%  4) ANSWER THE VALIDATION QUESTIONS
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' VALIDATION QUESTIONS\n');
fprintf('---------------------------------------------------------\n');

fprintf('Q1. Simulation completed without errors: %d\n', simCompleted);

fprintf('Q2. theta(0) = %.4f deg (expected ~+%.1f deg)\n', thHist(1), theta0_deg);

[~, i01] = min(abs(tSim - 0.1));
[~, i19] = min(abs(tSim - (t_dist_start - 0.1)));
fprintf('Q3. |theta| at t=0.10s = %.4f deg, |theta| at t=%.2fs (just before disturbance) = %.4f deg -> moved toward 0: %s\n', ...
    abs(thHist(i01)), tSim(i19), abs(thHist(i19)), mat2str(abs(thHist(i19)) < abs(thHist(i01))));

distMask = (tSim >= t_dist_start) & (tSim <= t_dist_end);
[~, iBefore] = min(abs(tSim - t_dist_start));
[~, iAfterPulse] = min(abs(tSim - t_dist_end));
fprintf('Q4. theta just before disturbance (t=%.2fs): %.4f deg\n', tSim(iBefore), thHist(iBefore));
fprintf('    theta at end of disturbance   (t=%.2fs): %.4f deg\n', tSim(iAfterPulse), thHist(iAfterPulse));
fprintf('    max |theta| during disturbance window:    %.4f deg\n', max(abs(thHist(distMask))));
fprintf('    max |F_LQR| during disturbance window:    %.4f N\n', max(abs(FLQR(distMask))));

postMask  = tSim > t_dist_end;
tPost     = tSim(postMask); thPost = thHist(postMask);
earlyPost = tPost <= (t_dist_end + 0.5);
latePost  = tPost >= (tEnd - 0.5);
meanEarly = mean(abs(thPost(earlyPost)));
meanLate  = mean(abs(thPost(latePost)));
fprintf('Q5. mean|theta| first 0.5s after disturbance = %.4f deg, last 0.5s = %.4f deg -> recovering: %d\n', ...
    meanEarly, meanLate, meanLate < meanEarly);
fprintf('    final theta(t=%.2fs) = %.4f deg\n', tSim(end), thHist(end));

[thMax, iThMax] = max(thHist);
[thMin, iThMin] = min(thHist);
fprintf('Q6. max theta = %.4f deg at t=%.3fs ; min theta = %.4f deg at t=%.3fs\n', ...
    thMax, tSim(iThMax), thMin, tSim(iThMin));

[FmaxAbs, iFmax] = max(abs(FLQR));
fprintf('Q7. max |F_LQR| = %.4f N at t=%.3fs (F_LQR value = %.4f N)\n', FmaxAbs, tSim(iFmax), FLQR(iFmax));

maxAbsX  = max(abs(xHist));
maxAbsXd = max(abs(xdHist));
allFiniteStates = all(isfinite(xHist)) && all(isfinite(xdHist)) && all(isfinite(thHist));
allFiniteForces = all(isfinite(FLQR)) && all(isfinite(Fdist)) && all(isfinite(Ftotal));
allFinite = allFiniteStates && allFiniteForces;
fprintf('Q8. max|x| = %.4f m, max|x_dot| = %.4f m/s, all signals finite: %d\n\n', maxAbsX, maxAbsXd, allFinite);

fprintf('---------------------------------------------------------\n');
fprintf(' SUMMARY OF KEY VALUES\n');
fprintf('---------------------------------------------------------\n');
fprintf('theta(0)          = %.4f deg\n', thHist(1));
fprintf('theta_max          = %.4f deg\n', thMax);
fprintf('theta_min          = %.4f deg\n', thMin);
fprintf('theta_final        = %.4f deg\n', thHist(end));
fprintf('max|F_LQR|         = %.4f N\n', FmaxAbs);
fprintf('max|x|             = %.4f m\n', maxAbsX);
fprintf('max|x_dot|         = %.4f m/s\n\n', maxAbsXd);

%% ------------------------------------------------------------------
%  5) FIGURE: SIX-PANEL SIMULINK RESPONSE
%  ------------------------------------------------------------------
shadeColor = [1.0 0.85 0.30];
padLim = @(v, lo, hi) [min(v(:)) - lo*(max(v(:))-min(v(:))+eps), max(v(:)) + hi*(max(v(:))-min(v(:))+eps)];

fig = figure('Color', 'w', 'Position', [50 30 1300 950]);

ax1 = subplot(3,2,1); hold(ax1,'on');
plot(ax1, tSim, thHist, 'r-', 'LineWidth', 2.0);
plot(ax1, [0 tEnd], [0 0], 'k--', 'LineWidth', 1.0);
grid(ax1,'on'); xlim(ax1,[0 tEnd]); ylim(ax1, padLim(thHist, 0.12, 0.25));
title(ax1, 'Pendulum Angle \theta(t)', 'FontWeight','bold'); xlabel(ax1,'Time [s]'); ylabel(ax1,'\theta [deg]');
legend(ax1, {'\theta(t)','Upright (0 deg)'}, 'Location','best', 'FontSize',8);

ax2 = subplot(3,2,2); hold(ax2,'on');
plot(ax2, tSim, xHist, 'b-', 'LineWidth', 1.8);
grid(ax2,'on'); xlim(ax2,[0 tEnd]); ylim(ax2, padLim(xHist, 0.15, 0.15));
title(ax2, 'Chassis Position x(t)', 'FontWeight','bold'); xlabel(ax2,'Time [s]'); ylabel(ax2,'x [m]');

ax3 = subplot(3,2,3); hold(ax3,'on');
plot(ax3, tSim, xdHist, 'Color', [0.1 0.5 0.2], 'LineWidth', 1.8);
grid(ax3,'on'); xlim(ax3,[0 tEnd]); ylim(ax3, padLim(xdHist, 0.15, 0.15));
title(ax3, 'Chassis Velocity x_{dot}(t)', 'FontWeight','bold'); xlabel(ax3,'Time [s]'); ylabel(ax3,'x_{dot} [m/s]');

ax4 = subplot(3,2,4); hold(ax4,'on');
plot(ax4, tSim, FLQR, 'Color', [0.55 0.2 0.65], 'LineWidth', 1.8);
grid(ax4,'on'); xlim(ax4,[0 tEnd]); ylim(ax4, padLim(FLQR, 0.15, 0.15));
title(ax4, 'LQR Control Force F_{LQR}(t)', 'FontWeight','bold'); xlabel(ax4,'Time [s]'); ylabel(ax4,'F_{LQR} [N]');

ax5 = subplot(3,2,5); hold(ax5,'on');
plot(ax5, tSim, Fdist, 'Color', [0.9 0.4 0.0], 'LineWidth', 2.0);
grid(ax5,'on'); xlim(ax5,[0 tEnd]); ylim(ax5, padLim(Fdist, 0.15, 0.3));
title(ax5, 'Disturbance Force F_{dist}(t)', 'FontWeight','bold'); xlabel(ax5,'Time [s]'); ylabel(ax5,'F_{dist} [N]');

ax6 = subplot(3,2,6); hold(ax6,'on');
plot(ax6, tSim, Ftotal, 'k-', 'LineWidth', 1.6);
grid(ax6,'on'); xlim(ax6,[0 tEnd]); ylim(ax6, padLim(Ftotal, 0.15, 0.15));
title(ax6, 'Total Force F_{total}(t)', 'FontWeight','bold'); xlabel(ax6,'Time [s]'); ylabel(ax6,'F_{total} [N]');

axList = [ax1 ax2 ax3 ax4 ax5 ax6];
for k = 1:numel(axList)
    axk = axList(k);
    yl = ylim(axk); ylim(axk, yl); hold(axk, 'on');
    p = patch(axk, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl(1) yl(1) yl(2) yl(2)], ...
        shadeColor, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    uistack(p, 'bottom');
end

sgtitle('Four-Wheeled Inverted Pendulum - Simulink LQR Response (raw Simulink output)', 'FontWeight','bold', 'FontSize',13);

%% ------------------------------------------------------------------
%  6) SAVE FIGURE AND DATA
%  ------------------------------------------------------------------
figFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_simulink_response.png');
exportgraphics(fig, figFile, 'Resolution', 200);
fprintf('Figure saved to: %s\n', figFile);

matFile = fullfile(resultsDir, ...
    'four_wheeled_inverted_pendulum_simulink_response.mat');
save(matFile, 'tSim', 'xHist', 'xdHist', 'thHist', 'thdHist', 'FLQR', 'Fdist', 'Ftotal', ...
    'theta0_deg', 't_dist_start', 't_dist_end', 'F_dist_mag', 'tEnd', ...
    'thMax', 'thMin', 'FmaxAbs', 'maxAbsX', 'maxAbsXd', 'simCompleted');
fprintf('Simulation data saved to: %s\n', matFile);

fprintf('\n=========================================================\n');
fprintf(' STEP 7B COMPLETE: Simulink model run and raw results reported.\n');
fprintf(' No MATLAB-vs-Simulink comparison performed. No parameters changed.\n');
fprintf('=========================================================\n');
