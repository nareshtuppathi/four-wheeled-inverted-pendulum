%% Four-Wheeled Inverted Pendulum - Quantitative Control Performance Analysis
% STEP 8 (completed as STEP 11.4) of the robotics portfolio project.
%
% Analyzes the ALREADY-VALIDATED disturbance-rejection simulation (Step 5,
% cross-checked against Simulink in Step 7C). No new simulation is run and
% no parameter (A, B, K, Q, R, physical parameters, initial condition,
% disturbance, controller, Simulink model) is changed here - this script
% only loads the existing saved results and computes metrics from them.

clear; clc; close all;
projectFolder = fullfile(getenv('USERPROFILE'), 'Documents', 'FourWheeledInvertedPendulum');

fprintf('=========================================================\n');
fprintf(' STEP 8: QUANTITATIVE CONTROL PERFORMANCE ANALYSIS\n');
fprintf('=========================================================\n');

%% ------------------------------------------------------------------
%  1) LOAD THE EXISTING, ALREADY-VALIDATED SIMULATION DATA
%  ------------------------------------------------------------------
% Step 5's MATLAB disturbance simulation: LQR + 2 N pulse, 2.0-2.2 s,
% cross-validated against Simulink in Step 7C. Nothing is recomputed.
dataFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_disturbance_response.mat');
assert(isfile(dataFile), 'Required data file not found: %s', dataFile);
D = load(dataFile);

tSim     = D.tSim;
thetaDeg = rad2deg(D.XSim(:,3));
xHist    = D.XSim(:,1);
xdHist   = D.XSim(:,2);
FLQR     = D.F_LQR;
t_dist_start = D.t_dist_start;  t_dist_end = D.t_dist_end;
F_dist_mag   = D.F_dist_mag;    theta0_deg  = D.theta0_deg;
tEnd = tSim(end);

fprintf('Loaded %d samples over t = [%.3f, %.3f] s from:\n  %s\n\n', numel(tSim), tSim(1), tSim(end), dataFile);

%% ------------------------------------------------------------------
%  2) SETTLING / RECOVERY CRITERIA (defined BEFORE computing results)
%  ------------------------------------------------------------------
% Band: |theta| <= 0.5 deg.
%
% Settling time (whole-run definition): the earliest time t* such that
% |theta(t)| <= band for ALL t in [t*, tEnd] - i.e. the LAST time the
% angle ever leaves the band, found by scanning backward from the end of
% the run. Because the 2 N disturbance re-excites the pendulum after its
% initial release, this single criterion naturally captures whichever
% excursion (the initial 5 deg release or the disturbance) settles last.
%
% Disturbance recovery time: the same crossing, but measured relative to
% the end of the disturbance pulse (t_dist_end = 2.2 s) rather than t=0,
% so it isolates how long recovery took after the external event.
band = 0.5;   % deg

outOfBand = find(abs(thetaDeg) > band);
if isempty(outOfBand)
    tSettle = tSim(1);
    settled = true;
elseif outOfBand(end) < numel(tSim)
    j = outOfBand(end);
    a1 = abs(thetaDeg(j)); a2 = abs(thetaDeg(j+1));
    tSettle = tSim(j) + (a1 - band)/(a1 - a2) * (tSim(j+1) - tSim(j));
    settled = true;
else
    tSettle = NaN;
    settled = false;   % never stays within the band through tEnd
end

if settled
    tRecover = tSettle - t_dist_end;
else
    tRecover = NaN;
end

%% ------------------------------------------------------------------
%  3) COMPUTE ALL 12 METRICS (directly from the loaded data, nothing invented)
%  ------------------------------------------------------------------
m1_initialAngle    = thetaDeg(1);
m2_maxPosAngle     = max(thetaDeg);
m3_minAngle        = min(thetaDeg);
m4_maxAbsAngle     = max(abs(thetaDeg));
m5_maxAbsForce     = max(abs(FLQR));
m6_maxAbsChassisX  = max(abs(xHist));
m7_maxAbsChassisV  = max(abs(xdHist));
m8_finalAngle      = thetaDeg(end);
m9_finalPosition   = xHist(end);
m10_finalVelocity  = xdHist(end);
m11_settlingTime   = tSettle;
m12_recoveryTime   = tRecover;

fprintf('---------------------------------------------------------\n');
fprintf(' SETTLING / RECOVERY METHOD AND RESULT\n');
fprintf('---------------------------------------------------------\n');
fprintf('Band used: |theta| <= %.1f deg (fixed before computing results)\n', band);
fprintf('Settling time definition: earliest t* such that |theta(t)| <= %.1f deg\n', band);
fprintf('  for ALL t in [t*, tEnd=%.1fs] (captures whichever excursion - the\n', tEnd);
fprintf('  initial 5 deg release or the disturbance - settles last).\n');
if settled
    fprintf('  Result: t_settle = %.4f s\n', tSettle);
else
    fprintf('  Result: NOT SETTLED - |theta| exceeds %.1f deg at some point and never\n', band);
    fprintf('  returns within the band through t=%.1fs. No settling time reported.\n', tEnd);
end
fprintf('Recovery time definition: t_settle - t_dist_end (t_dist_end = %.1fs)\n', t_dist_end);
if settled
    fprintf('  Result: t_recover = %.4f s after the disturbance ends\n\n', tRecover);
else
    fprintf('  Result: NOT AVAILABLE (system never settles within the band)\n\n');
end

fprintf('---------------------------------------------------------\n');
fprintf(' ALL 12 METRICS (calculated from existing simulation data)\n');
fprintf('---------------------------------------------------------\n');
fprintf('1.  Initial pendulum angle              = %8.4f deg\n', m1_initialAngle);
fprintf('2.  Maximum positive pendulum angle     = %8.4f deg\n', m2_maxPosAngle);
fprintf('3.  Minimum pendulum angle              = %8.4f deg\n', m3_minAngle);
fprintf('4.  Maximum absolute pendulum angle     = %8.4f deg\n', m4_maxAbsAngle);
fprintf('5.  Maximum absolute LQR control force  = %8.4f N\n', m5_maxAbsForce);
fprintf('6.  Maximum absolute chassis displacement = %6.4f m\n', m6_maxAbsChassisX);
fprintf('7.  Maximum absolute chassis velocity   = %8.4f m/s\n', m7_maxAbsChassisV);
fprintf('8.  Final pendulum angle                = %8.4f deg\n', m8_finalAngle);
fprintf('9.  Final chassis position              = %8.4f m\n', m9_finalPosition);
fprintf('10. Final chassis velocity              = %8.4f m/s\n', m10_finalVelocity);
if settled
    fprintf('11. Settling time (|theta|<=%.1fdeg band)  = %6.4f s\n', band, m11_settlingTime);
    fprintf('12. Disturbance recovery time            = %6.4f s\n\n', m12_recoveryTime);
else
    fprintf('11. Settling time (|theta|<=%.1fdeg band)  = NOT SETTLED by t=%.1fs\n', band, tEnd);
    fprintf('12. Disturbance recovery time            = NOT AVAILABLE\n\n');
end

%% ------------------------------------------------------------------
%  4) PERFORMANCE TABLE
%  ------------------------------------------------------------------
metricNames = { ...
    'Initial pendulum angle'; 'Maximum positive pendulum angle'; 'Minimum pendulum angle'; ...
    'Maximum absolute pendulum angle'; 'Maximum absolute LQR control force'; ...
    'Maximum absolute chassis displacement'; 'Maximum absolute chassis velocity'; ...
    'Final pendulum angle'; 'Final chassis position'; 'Final chassis velocity'; ...
    'Settling time (|theta| <= 0.5 deg band, sustained to end)'; ...
    'Disturbance recovery time (after t=2.2 s)' };
units = {'deg';'deg';'deg';'deg';'N';'m';'m/s';'deg';'m';'m/s';'s';'s'};
values = [m1_initialAngle; m2_maxPosAngle; m3_minAngle; m4_maxAbsAngle; m5_maxAbsForce; ...
    m6_maxAbsChassisX; m7_maxAbsChassisV; m8_finalAngle; m9_finalPosition; m10_finalVelocity; ...
    m11_settlingTime; m12_recoveryTime];
definitions = { ...
    'theta(t=0), from the initial condition'; ...
    'max(theta(t)) over the full 0-5 s run'; ...
    'min(theta(t)) over the full 0-5 s run'; ...
    'max(abs(theta(t))) over the full 0-5 s run'; ...
    'max(abs(F_LQR(t))), F_LQR = -K*X'; ...
    'max(abs(x(t))) over the full 0-5 s run'; ...
    'max(abs(x_dot(t))) over the full 0-5 s run'; ...
    'theta(t=5s), end of the simulation'; ...
    'x(t=5s), end of the simulation'; ...
    'x_dot(t=5s), end of the simulation'; ...
    'Earliest t* with |theta(t)|<=0.5deg for all t in [t*,5s]'; ...
    'Settling time minus t_dist_end (2.2 s)' };
T = table(metricNames, values, units, definitions, ...
    'VariableNames', {'Metric','Value','Unit','Definition'});

fprintf('---------------------------------------------------------\n');
fprintf(' PERFORMANCE METRICS TABLE\n');
fprintf('---------------------------------------------------------\n');
disp(T);

%% ------------------------------------------------------------------
%  5) SAVE CSV AND MAT
%     NOTE: uses the NEW filename '...performance_analysis.csv' so the
%     existing Step 6 file '...performance_metrics.csv' is NOT overwritten.
%  ------------------------------------------------------------------
csvFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_performance_analysis.csv');
writetable(T, csvFile);
fprintf('\nPerformance table saved to: %s\n', csvFile);

matFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_performance_analysis.mat');
save(matFile, 'tSim', 'thetaDeg', 'xHist', 'xdHist', 'FLQR', 't_dist_start', 't_dist_end', ...
    'F_dist_mag', 'theta0_deg', 'band', 'settled', ...
    'm1_initialAngle','m2_maxPosAngle','m3_minAngle','m4_maxAbsAngle','m5_maxAbsForce', ...
    'm6_maxAbsChassisX','m7_maxAbsChassisV','m8_finalAngle','m9_finalPosition', ...
    'm10_finalVelocity','m11_settlingTime','m12_recoveryTime','T');
fprintf('Performance data saved to: %s\n', matFile);

%% ------------------------------------------------------------------
%  6) PROFESSIONAL QUANTITATIVE SUMMARY FIGURE
%  ------------------------------------------------------------------
shadeColor = [1.0 0.85 0.30];
padLim = @(v, lo, hi) [min(v(:)) - lo*(max(v(:))-min(v(:))+eps), max(v(:)) + hi*(max(v(:))-min(v(:))+eps)];

fig = figure('Color', 'w', 'Position', [50 30 1300 900]);

% --- Panel 1 (top-left): pendulum angle with settling band and markers ---
% Shading patches are drawn FIRST (before the data lines) so they sit
% behind the traces by draw order - this avoids uistack, which does not
% support arbitrary child reordering on axes using yyaxis (Panel 3).
ax1 = subplot(2,2,1); hold(ax1, 'on');
yl1 = padLim(thetaDeg, 0.12, 0.3);
patch(ax1, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl1(1) yl1(1) yl1(2) yl1(2)], ...
    shadeColor, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax1, tSim, thetaDeg, 'r-', 'LineWidth', 2.0, 'DisplayName', '\theta(t)');
plot(ax1, [0 tEnd], [0 0], 'k--', 'LineWidth', 1.0, 'DisplayName', 'Upright (0 deg)');
plot(ax1, [0 tEnd], [band band], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'DisplayName', sprintf('\\pm%.1f deg band', band));
plot(ax1, [0 tEnd], [-band -band], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');
if settled
    xline(ax1, m11_settlingTime, '-', 'Color', [0.1 0.5 0.2], 'LineWidth', 1.5, 'DisplayName', sprintf('Settled at %.3fs', m11_settlingTime));
end
grid(ax1, 'on'); xlim(ax1, [0 tEnd]); ylim(ax1, yl1);
title(ax1, 'Pendulum Angle \theta(t)', 'FontWeight', 'bold'); xlabel(ax1, 'Time [s]'); ylabel(ax1, '\theta [deg]');
legend(ax1, 'Location', 'best', 'FontSize', 8);

% --- Panel 2 (top-right): LQR control force ---
ax2 = subplot(2,2,2); hold(ax2, 'on');
yl2 = padLim(FLQR, 0.12, 0.2);
patch(ax2, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl2(1) yl2(1) yl2(2) yl2(2)], ...
    shadeColor, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax2, tSim, FLQR, 'Color', [0.55 0.2 0.65], 'LineWidth', 1.8, 'DisplayName', 'F_{LQR}(t)');
grid(ax2, 'on'); xlim(ax2, [0 tEnd]); ylim(ax2, yl2);
title(ax2, 'LQR Control Force F_{LQR}(t)', 'FontWeight', 'bold'); xlabel(ax2, 'Time [s]'); ylabel(ax2, 'F_{LQR} [N]');
legend(ax2, 'Location', 'best', 'FontSize', 8);

% --- Panel 3 (bottom-left): chassis position and velocity ---
ax3 = subplot(2,2,3); hold(ax3, 'on');
yl3 = padLim(xHist, 0.12, 0.2);
patch(ax3, [t_dist_start t_dist_end t_dist_end t_dist_start], [yl3(1) yl3(1) yl3(2) yl3(2)], ...
    shadeColor, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
yyaxis(ax3, 'left');
plot(ax3, tSim, xHist, 'b-', 'LineWidth', 1.8);
ylabel(ax3, 'x [m]'); ylim(ax3, yl3);
yyaxis(ax3, 'right');
plot(ax3, tSim, xdHist, 'Color', [0.1 0.5 0.2], 'LineWidth', 1.4, 'LineStyle', '--');
ylabel(ax3, 'x_{dot} [m/s]'); ylim(ax3, padLim(xdHist, 0.12, 0.2));
grid(ax3, 'on'); xlim(ax3, [0 tEnd]);
title(ax3, 'Chassis Position and Velocity', 'FontWeight', 'bold'); xlabel(ax3, 'Time [s]');
legend(ax3, {'x(t)', 'x_{dot}(t)'}, 'Location', 'best', 'FontSize', 8);

% --- Panel 4 (bottom-right): key metrics as a text summary panel ---
ax4 = subplot(2,2,4);
axis(ax4, 'off');
summaryLines = { ...
    sprintf('Initial \\theta            = %+7.4f deg', m1_initialAngle); ...
    sprintf('Max \\theta               = %+7.4f deg', m2_maxPosAngle); ...
    sprintf('Min \\theta               = %+7.4f deg', m3_minAngle); ...
    sprintf('Max |\\theta|             = %7.4f deg', m4_maxAbsAngle); ...
    sprintf('Max |F_{LQR}|            = %7.4f N', m5_maxAbsForce); ...
    sprintf('Max |x|                  = %7.4f m', m6_maxAbsChassisX); ...
    sprintf('Max |x_{dot}|            = %7.4f m/s', m7_maxAbsChassisV); ...
    sprintf('Final \\theta             = %+7.4f deg', m8_finalAngle); ...
    sprintf('Final x                  = %+7.4f m', m9_finalPosition); ...
    sprintf('Final x_{dot}            = %+7.4f m/s', m10_finalVelocity) };
if settled
    summaryLines{end+1} = sprintf('Settling time (%.1f{\\circ} band) = %6.3f s', band, m11_settlingTime);
    summaryLines{end+1} = sprintf('Disturbance recovery    = %6.3f s', m12_recoveryTime);
else
    summaryLines{end+1} = sprintf('Settling time (%.1f{\\circ} band) = not settled by %.1fs', band, tEnd);
    summaryLines{end+1} = 'Disturbance recovery    = not available';
end
text(ax4, 0.02, 0.98, 'Key Performance Metrics', 'Units', 'normalized', ...
    'FontWeight', 'bold', 'FontSize', 13, 'VerticalAlignment', 'top');
text(ax4, 0.02, 0.88, strjoin(summaryLines, newline), 'Units', 'normalized', ...
    'FontSize', 11, 'FontName', 'Consolas', 'VerticalAlignment', 'top');

sgtitle('Four-Wheeled Inverted Pendulum - Quantitative Control Performance Analysis', ...
    'FontWeight', 'bold', 'FontSize', 14);

pngFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_performance_analysis.png');
exportgraphics(fig, pngFile, 'Resolution', 200);
fprintf('\nFigure saved to: %s\n', pngFile);

%% ------------------------------------------------------------------
%  7) FINAL VERIFICATION
%  ------------------------------------------------------------------
fprintf('\n---------------------------------------------------------\n');
fprintf(' FINAL VERIFICATION\n');
fprintf('---------------------------------------------------------\n');
fprintf('Script executed without errors: 1\n');
fprintf('PNG exists: %d (%s)\n', isfile(pngFile), pngFile);
fprintf('MAT exists: %d (%s)\n', isfile(matFile), matFile);
fprintf('CSV exists: %d (%s)\n', isfile(csvFile), csvFile);

fprintf('\n=========================================================\n');
fprintf(' STEP 8 COMPLETE.\n');
fprintf('=========================================================\n');
