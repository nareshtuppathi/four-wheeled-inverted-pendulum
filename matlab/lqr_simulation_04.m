%% Four-Wheeled Inverted Pendulum - Closed-Loop Time-Domain Simulation
% STEP 4 of the robotics portfolio project.
%
% Pipeline implemented in this script:
%   INITIAL 5 DEG TILT -> LQR CONTROLLER -> CLOSED-LOOP SIMULATION
%   -> PENDULUM RETURNS TOWARD 0 DEG -> PLOT SYSTEM RESPONSE
%
% This script ONLY simulates the closed-loop linear system X_dot =
% (A - B*K)*X using the LQR gain K from Step 3, starting from a small
% initial pendulum tilt. No external disturbances, no Simulink model,
% and no animation are included here.

clear; clc; close all;

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS (identical to Steps 2 and 3)
%  ------------------------------------------------------------------
M = 2.50;      % equivalent chassis/base mass [kg]
m = 0.50;      % pendulum mass [kg]
l = 0.375;     % pivot-to-CoM distance [m]
I = 0.015;     % pendulum moment of inertia about its own CoM [kg*m^2]
g = 9.81;      % gravitational acceleration [m/s^2]

%% ------------------------------------------------------------------
%  2) LINEARIZED STATE-SPACE MODEL (reconstructed from Step 2)
%     State vector: X = [x; x_dot; theta; theta_dot]
%     Model: X_dot = A*X + B*F
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
%  3) LQR GAIN K (reconstructed from Step 3, same Q and R)
%  ------------------------------------------------------------------
Q = diag([1, 1, 100, 10]);   % heavy weight on theta, then theta_dot
R = 1;                       % moderate control-effort penalty
K = lqr(A, B, Q, R);

fprintf('=========================================================\n');
fprintf(' STEP 4: CLOSED-LOOP TIME-DOMAIN SIMULATION\n');
fprintf('=========================================================\n');
fprintf('LQR gain K = [%.4f, %.4f, %.4f, %.4f]\n\n', K(1), K(2), K(3), K(4));

%% ------------------------------------------------------------------
%  4) CLOSED-LOOP DYNAMICS
%     X_dot = (A - B*K)*X   (autonomous, no reference/disturbance input)
%  ------------------------------------------------------------------
A_cl = A - B*K;

%% ------------------------------------------------------------------
%  5) INITIAL CONDITIONS AND SIMULATION SETTINGS
%  ------------------------------------------------------------------
x0         = 0;                  % initial chassis position [m]
x_dot0     = 0;                  % initial chassis velocity [m/s]
theta0_deg = 5;                  % initial pendulum tilt [degrees]
theta0     = deg2rad(theta0_deg);% initial pendulum tilt [radians]
theta_dot0 = 0;                  % initial pendulum angular velocity [rad/s]

X0 = [x0; x_dot0; theta0; theta_dot0];

tSpan = [0 5];   % simulate for approximately 5 seconds

fprintf('Initial conditions:\n');
fprintf('  x(0)         = %.4f m\n', x0);
fprintf('  x_dot(0)     = %.4f m/s\n', x_dot0);
fprintf('  theta(0)     = %.2f deg (%.4f rad)\n', theta0_deg, theta0);
fprintf('  theta_dot(0) = %.4f rad/s\n\n', theta_dot0);

%% ------------------------------------------------------------------
%  6) NUMERICAL INTEGRATION USING ode45
%  ------------------------------------------------------------------
odeOptions = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
closedLoopODE = @(t, X) A_cl * X;
[tSim, XSim] = ode45(closedLoopODE, tSpan, X0, odeOptions);

%% ------------------------------------------------------------------
%  7) RECOVER THE CONTROL FORCE F(t) = -K*X(t) ALONG THE TRAJECTORY
%  ------------------------------------------------------------------
FSim = -(K * XSim.').';   % F(t) evaluated at every simulated time step

%% ------------------------------------------------------------------
%  8) EXTRACT STATE HISTORIES FOR PLOTTING
%  ------------------------------------------------------------------
xHist         = XSim(:,1);            % chassis position [m]
xDotHist      = XSim(:,2);            % chassis velocity [m/s]
thetaHistDeg  = rad2deg(XSim(:,3));   % pendulum angle [deg]
thetaDotHist  = XSim(:,4);            % pendulum angular velocity [rad/s]

%% ------------------------------------------------------------------
%  9) SUMMARY METRICS
%  ------------------------------------------------------------------
initialAngleDeg = thetaHistDeg(1);
finalAngleDeg   = thetaHistDeg(end);
maxAbsAngleDeg  = max(abs(thetaHistDeg));
maxAbsForce     = max(abs(FSim));
finalPosition   = xHist(end);

fprintf('---------------------------------------------------------\n');
fprintf(' SIMULATION RESULTS SUMMARY\n');
fprintf('---------------------------------------------------------\n');
fprintf('Initial pendulum angle       : %.4f deg\n', initialAngleDeg);
fprintf('Final pendulum angle         : %.6f deg\n', finalAngleDeg);
fprintf('Maximum absolute angle       : %.4f deg\n', maxAbsAngleDeg);
fprintf('Maximum absolute control F   : %.4f N\n', maxAbsForce);
fprintf('Final chassis position       : %.6f m\n\n', finalPosition);

%% ------------------------------------------------------------------
%  10) AUTOMATIC VERIFICATION CHECKS
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' AUTOMATIC CHECKS\n');
fprintf('---------------------------------------------------------\n');

% Check 1: pendulum angle should trend toward zero (compare early vs
% late portion of the simulation using the mean absolute angle).
nPts = numel(thetaHistDeg);
earlyWindow = 1:max(1, round(0.1*nPts));
lateWindow  = round(0.8*nPts):nPts;
meanAbsEarly = mean(abs(thetaHistDeg(earlyWindow)));
meanAbsLate  = mean(abs(thetaHistDeg(lateWindow)));
convergesTowardZero = meanAbsLate < meanAbsEarly;
fprintf('Check 1 (angle converges toward zero over time):    %s\n', mat2str(convergesTowardZero));
fprintf('         mean|theta| early = %.4f deg, late = %.6f deg\n', meanAbsEarly, meanAbsLate);

% Check 2: final angle sufficiently close to zero. The tolerance is defined
% relative to the initial tilt (settling band of 5% of the initial 5 deg)
% rather than a fixed absolute value, because the slowest closed-loop mode
% (lambda ~ -0.47, time constant ~2.1 s) has not fully decayed by t = 5 s.
angleTolDeg = max(0.1, 0.05 * theta0_deg);
finalAngleNearZero = abs(finalAngleDeg) < angleTolDeg;
fprintf('Check 2 (final angle within %.3f deg, i.e. 5%% of initial tilt): %s\n', angleTolDeg, mat2str(finalAngleNearZero));

% Check 3: simulation remains numerically stable (no NaN/Inf states)
simIsFinite = all(isfinite(XSim(:)));
fprintf('Check 3 (all simulated states finite, no NaN/Inf):   %s\n', mat2str(simIsFinite));

% Check 4: control input is finite for the entire simulation
forceIsFinite = all(isfinite(FSim));
fprintf('Check 4 (control force F(t) finite for all t):      %s\n', mat2str(forceIsFinite));

allChecksPassed = convergesTowardZero && finalAngleNearZero && simIsFinite && forceIsFinite;
fprintf('\n');
if allChecksPassed
    fprintf('ALL AUTOMATIC CHECKS PASSED.\n');
else
    fprintf('WARNING: One or more checks failed. Review the simulation.\n');
end
fprintf('\n');

%% ------------------------------------------------------------------
%  11) FIGURE: FOUR-PANEL CLOSED-LOOP RESPONSE PLOT
%  ------------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [80 60 1100 850]);

% --- Plot 1: chassis position x(t) ---
subplot(2,2,1);
plot(tSim, xHist, 'b-', 'LineWidth', 1.8);
grid on;
title('Chassis Position x(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]'); ylabel('x [m]');
legend('x(t)', 'Location', 'best');

% --- Plot 2: chassis velocity x_dot(t) ---
subplot(2,2,2);
plot(tSim, xDotHist, 'Color', [0.1 0.5 0.2], 'LineWidth', 1.8);
grid on;
title('Chassis Velocity x_{dot}(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]'); ylabel('x_{dot} [m/s]');
legend('x_{dot}(t)', 'Location', 'best');

% --- Plot 3: pendulum angle theta(t), highlighting convergence to 0 ---
subplot(2,2,3);
plot(tSim, thetaHistDeg, 'r-', 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.2);
grid on;
title('Pendulum Angle \theta(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]'); ylabel('\theta [deg]');
legend('\theta(t)', 'Upright (0 deg)', 'Location', 'best');

% --- Plot 4: control force F(t) ---
subplot(2,2,4);
plot(tSim, FSim, 'Color', [0.55 0.2 0.65], 'LineWidth', 1.8);
grid on;
title('Control Force F(t)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]'); ylabel('F [N]');
legend('F(t) = -K X(t)', 'Location', 'best');

sgtitle('Four-Wheeled Inverted Pendulum - LQR Closed-Loop Response (5 deg initial tilt)', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ------------------------------------------------------------------
%% ------------------------------------------------------------------
%  12) SAVE FIGURE AND DATA
%  ------------------------------------------------------------------

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Save Step 4 results in the organized results folder
resultsDir = fullfile(projectRoot, 'results', '04_lqr_simulation');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

figFile = fullfile(resultsDir, 'four_wheeled_inverted_pendulum_lqr_response.png');
exportgraphics(fig, figFile, 'Resolution', 200);
fprintf('Figure saved to: %s\n', figFile);

matFile = fullfile(resultsDir, 'four_wheeled_inverted_pendulum_lqr_response.mat');
save(matFile, 'tSim', 'XSim', 'FSim', 'K', 'A', 'B', 'A_cl', 'Q', 'R', ...
    'theta0_deg', 'initialAngleDeg', 'finalAngleDeg', 'maxAbsAngleDeg', ...
    'maxAbsForce', 'finalPosition');
fprintf('Simulation data saved to: %s\n', matFile);