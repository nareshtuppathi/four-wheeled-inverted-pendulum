%% Build script for the Simulink model four_wheeled_inverted_pendulum_lqr.slx
% STEP 7 of the robotics portfolio project.
%
% Programmatically builds the Simulink implementation of the SAME linear
% LQR system used in the MATLAB scripts (Steps 2-6):
%       X_dot   = A*X + B*F_total,   X = [x; x_dot; theta; theta_dot]
%       F_LQR   = -K*X
%       F_total = F_LQR + F_disturbance   (2 N for 2.0 <= t <= 2.2 s)
% Physical parameters, A, B, Q, R and K are NOT changed: they are defined by
% the model's PreLoadFcn/InitFcn callbacks with exactly the same formulas.
% No nonlinear dynamics, extra controllers, hardware or ROS are included.

clear; clc;

mdl = 'four_wheeled_inverted_pendulum_lqr';

% Project root = one level above the matlab folder
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Simulink model goes in the simulink folder
simulinkDir = fullfile(projectRoot, 'simulink');
if ~exist(simulinkDir, 'dir')
    mkdir(simulinkDir);
end

% Model diagram image goes in the results folder
resultsDir = fullfile(projectRoot, 'results', '07_simulink');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

slxFile = fullfile(simulinkDir, [mdl '.slx']);
pngFile = fullfile(resultsDir, 'four_wheeled_inverted_pendulum_simulink_model.png');
blk       = @(name) [mdl '/' name];

%% ------------------------------------------------------------------
%  1) MODEL PARAMETERS (same values and formulas as Steps 2-6)
%  ------------------------------------------------------------------
% These lines become the model callbacks so the .slx is self-contained:
% opening or running the model always recreates A, B, K and X0 in the base
% workspace. The initial 5 deg tilt is converted to radians for X0.
cbLines = {
    '% Four-Wheeled Inverted Pendulum - parameters (identical to the MATLAB scripts)'
    'M = 2.50;  m = 0.50;  l = 0.375;  I = 0.015;  g = 9.81;'
    'D = M*(I + m*l^2) - (m*l)^2;'
    'A = [0 1 0 0; 0 0 -(g*m^2*l^2)/D 0; 0 0 0 1; 0 0 (M*g*m*l)/D 0];'
    'B = [0; (m*l^2 + I)/D; 0; -(m*l)/D];'
    'Q = diag([1, 1, 100, 10]);  R = 1;'
    'K = lqr(A, B, Q, R);'
    'theta0_deg = 5;  X0 = [0; 0; deg2rad(theta0_deg); 0];'
    'F_dist_mag = 2.0;  t_dist_start = 2.0;  t_dist_end = 2.2;  tEnd = 5.0;'
    };
cbCode = strjoin(cbLines, newline);
evalin('base', cbCode);   % define the variables now so block parameters resolve

%% ------------------------------------------------------------------
%  2) CREATE MODEL AND SOLVER CONFIGURATION
%  ------------------------------------------------------------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% Same numerical settings as the MATLAB ode45 simulations: ode45, RelTol = AbsTol = 1e-9,
% 0-5 s. MaxStep = 0.005 s matches the 200 samples/s output density used in MATLAB.
set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'ode45', ...
    'RelTol', '1e-9', 'AbsTol', '1e-9', ...
    'StartTime', '0', 'StopTime', 'tEnd', 'MaxStep', '0.005', ...
    'ReturnWorkspaceOutputs', 'on');

%% ------------------------------------------------------------------
%  3) EXTERNAL DISTURBANCE SUBSYSTEM
%     F_disturbance = +2 N at t = 2.0 s (Pulse Start), then 0 again at 2.2 s
%     (Pulse End adds -2 N). Step blocks use zero-crossing detection so the
%     solver lands exactly on both edges.
%  ------------------------------------------------------------------
dist = blk('External Disturbance');
add_block('simulink/Ports & Subsystems/Subsystem', dist, 'Position', [60 50 200 100]);
delete_line(dist, 'In1/1', 'Out1/1');
delete_block([dist '/In1']);
set_param([dist '/Out1'], 'Name', 'F_disturbance');
add_block('simulink/Sources/Step', [dist '/Pulse Start'], 'Time', 't_dist_start', ...
    'Before', '0', 'After', 'F_dist_mag', 'Position', [30 25 70 55]);
add_block('simulink/Sources/Step', [dist '/Pulse End'], 'Time', 't_dist_end', ...
    'Before', '0', 'After', '-F_dist_mag', 'Position', [30 85 70 115]);
add_block('simulink/Math Operations/Sum', [dist '/Sum'], 'Inputs', '++', ...
    'IconShape', 'rectangular', 'Position', [130 50 155 90]);
set_param([dist '/F_disturbance'], 'Position', [210 63 240 77]);
add_line(dist, 'Pulse Start/1', 'Sum/1');
add_line(dist, 'Pulse End/1', 'Sum/2');
add_line(dist, 'Sum/1', 'F_disturbance/1');
set_param(dist, 'BackgroundColor', 'orange');

%% ------------------------------------------------------------------
%  4) LQR CONTROLLER SUBSYSTEM:  F_LQR = -K*X
%  ------------------------------------------------------------------
ctrl = blk('LQR Controller');
add_block('simulink/Ports & Subsystems/Subsystem', ctrl, 'Position', [430 240 570 290]);
set_param(ctrl, 'Orientation', 'left');   % flipped so the feedback path reads right-to-left
delete_line(ctrl, 'In1/1', 'Out1/1');
set_param([ctrl '/In1'],  'Name', 'X');
set_param([ctrl '/Out1'], 'Name', 'F_LQR');
add_block('simulink/Math Operations/Gain', [ctrl '/State Feedback Gain -K'], ...
    'Gain', '-K', 'Multiplication', 'Matrix(K*u)', 'Position', [120 25 230 65]);
set_param([ctrl '/X'],     'Position', [30 38 60 52]);
set_param([ctrl '/F_LQR'], 'Position', [290 38 320 52]);
add_line(ctrl, 'X/1', 'State Feedback Gain -K/1');
add_line(ctrl, 'State Feedback Gain -K/1', 'F_LQR/1');
set_param(ctrl, 'BackgroundColor', 'lightBlue');

%% ------------------------------------------------------------------
%  5) TOTAL FORCE SUM, STATE-SPACE PLANT AND STATE DEMUX
%  ------------------------------------------------------------------
add_block('simulink/Math Operations/Sum', blk('Total Force'), 'Inputs', '++', ...
    'IconShape', 'rectangular', 'Position', [300 105 330 155]);

% Plant: X_dot = A*X + B*F_total. C = eye(4) makes the output the full state vector
% [x; x_dot; theta; theta_dot] (theta in radians); D = 0. Initial state X0 (rad).
add_block('simulink/Continuous/State-Space', blk('State-Space Plant'), ...
    'A', 'A', 'B', 'B', 'C', 'eye(4)', 'D', 'zeros(4,1)', 'X0', 'X0', ...
    'Position', [430 105 570 155], 'BackgroundColor', 'yellow');

add_block('simulink/Signal Routing/Demux', blk('State Demux'), 'Outputs', '4', ...
    'Position', [700 30 705 430]);

%% ------------------------------------------------------------------
%  6) SCOPES AND LOGGING (To Workspace, Timeseries format)
%  ------------------------------------------------------------------
yPort = [80 180 280 380];   % vertical position of each demux output
scopeNames = {'Chassis Position', 'Chassis Velocity', 'Pendulum Angle', 'Pendulum Angular Velocity'};
logNames   = {'Log x', 'Log x_dot', 'Log theta_deg', 'Log theta_dot'};
logVars    = {'x_sim', 'xdot_sim', 'theta_deg_sim', 'thetadot_sim'};
for k = 1:4
    y = yPort(k);
    add_block('simulink/Sinks/Scope', blk(scopeNames{k}), 'NumInputPorts', '1', ...
        'Position', [840 y-45 880 y-15]);
    add_block('simulink/Sinks/To Workspace', blk(logNames{k}), 'VariableName', logVars{k}, ...
        'SaveFormat', 'Timeseries', 'Position', [840 y+5 920 y+35]);
end

% Pendulum angle is converted from radians to degrees for visualization and logging
add_block('simulink/Math Operations/Gain', blk('Radians to Degrees'), 'Gain', '180/pi', ...
    'Position', [745 268 795 292]);

% Force logging: LQR force, disturbance force and total force
add_block('simulink/Sinks/To Workspace', blk('Log F_dist'),  'VariableName', 'F_dist_sim',  ...
    'SaveFormat', 'Timeseries', 'Position', [250 10 335 35]);
add_block('simulink/Sinks/To Workspace', blk('Log F_total'), 'VariableName', 'F_total_sim', ...
    'SaveFormat', 'Timeseries', 'Position', [350 50 435 75]);
add_block('simulink/Sinks/To Workspace', blk('Log F_LQR'),   'VariableName', 'F_LQR_sim',   ...
    'SaveFormat', 'Timeseries', 'Position', [290 310 375 335]);

%% ------------------------------------------------------------------
%  7) CONNECTIONS
%  ------------------------------------------------------------------
ln = @(src, dst) add_line(mdl, src, dst, 'autorouting', 'smart');
ln('External Disturbance/1', 'Total Force/1');
ln('LQR Controller/1',       'Total Force/2');
ln('Total Force/1',          'State-Space Plant/1');
ln('State-Space Plant/1',    'State Demux/1');
ln('State-Space Plant/1',    'LQR Controller/1');      % state feedback
ln('State Demux/1', 'Chassis Position/1');
ln('State Demux/1', 'Log x/1');
ln('State Demux/2', 'Chassis Velocity/1');
ln('State Demux/2', 'Log x_dot/1');
ln('State Demux/3', 'Radians to Degrees/1');
ln('Radians to Degrees/1', 'Pendulum Angle/1');
ln('Radians to Degrees/1', 'Log theta_deg/1');
ln('State Demux/4', 'Pendulum Angular Velocity/1');
ln('State Demux/4', 'Log theta_dot/1');
ln('External Disturbance/1', 'Log F_dist/1');
ln('Total Force/1',          'Log F_total/1');
ln('LQR Controller/1',       'Log F_LQR/1');

%% ------------------------------------------------------------------
%  8) SCOPE DISPLAY SETTINGS AND ANNOTATIONS
%  ------------------------------------------------------------------
scopeTitles = {'Chassis Position x [m]', 'Chassis Velocity x_dot [m/s]', ...
    'Pendulum Angle theta [deg]', 'Pendulum Angular Velocity theta_dot [rad/s]'};
scopeYLabel = {'x [m]', 'x_dot [m/s]', 'theta [deg]', 'theta_dot [rad/s]'};
for k = 1:4
    set_param(blk(scopeNames{k}), 'OpenAtSimulationStart', 'off');
    try
        cfg = get_param(blk(scopeNames{k}), 'ScopeConfiguration');
        cfg.Title = scopeTitles{k};
        cfg.YLabel = scopeYLabel{k};
        cfg.ShowGrid = true;
    catch scopeErr
        fprintf('Note: scope display settings skipped (%s)\n', scopeErr.message);
    end
end
% The pendulum-angle scope opens automatically when the model is run interactively.
set_param(blk('Pendulum Angle'), 'OpenAtSimulationStart', 'on');

try
    a1 = Simulink.Annotation(mdl, 'Four-Wheeled Inverted Pendulum - LQR state feedback (linear model)');
    a1.Position = [60 370];
    a1.FontSize = 14;
    a1.FontWeight = 'bold';
    a2 = Simulink.Annotation(mdl, sprintf(['X_dot = A*X + B*F_total,  X = [x; x_dot; theta; theta_dot]\n' ...
        'F_LQR = -K*X,  F_total = F_LQR + F_disturbance\n' ...
        'Disturbance: 2 N for 2.0 s <= t <= 2.2 s;  X(0) = [0; 0; 5 deg; 0]']));
    a2.Position = [60 400];
    a2.FontSize = 11;
catch annErr
    fprintf('Note: annotations skipped (%s)\n', annErr.message);
end

%% ------------------------------------------------------------------
%  9) MODEL CALLBACKS, UPDATE DIAGRAM, SAVE
%  ------------------------------------------------------------------
set_param(mdl, 'PreLoadFcn', cbCode);   % runs when the model is loaded
set_param(mdl, 'InitFcn',    cbCode);   % runs at the start of every simulation

set_param(mdl, 'SimulationCommand', 'update');   % compile check - errors surface here
save_system(mdl, slxFile);
fprintf('Model saved: %s\n', slxFile);

%% ------------------------------------------------------------------
%  10) EXPORT BLOCK DIAGRAM SCREENSHOT FOR THE README
%  ------------------------------------------------------------------
set_param(mdl, 'ZoomFactor', 'FitSystem');
drawnow; pause(1);
print(['-s' mdl], '-dpng', '-r200', pngFile);
fprintf('Block diagram image saved: %s\n', pngFile);
close_system(mdl, 0);

fprintf('\n=========================================================\n');
fprintf(' STEP 7 BUILD COMPLETE: %s.slx created and saved.\n', mdl);
fprintf('=========================================================\n');
