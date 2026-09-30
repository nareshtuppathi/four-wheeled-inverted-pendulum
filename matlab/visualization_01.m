%% Four-Wheeled Inverted Pendulum - Physical Model Visualization
% Step 1 of robotics portfolio project: PHYSICAL MODEL + 3D VISUALIZATION ONLY.
% No dynamics, equations of motion, or control are implemented in this file.
%
% Revision: presentation/readability pass (figure sizing, label placement,
% camera angle, font sizing). Physical model and parameters are unchanged.

clear; close all; clc;

%% ------------------------------------------------------------------
%  PORTABLE PROJECT PATHS (works from any clone location, any machine)
%  ------------------------------------------------------------------
scriptDir  = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);
figuresDir = fullfile(projectDir, 'results', 'figures');
if ~exist(figuresDir, 'dir'), mkdir(figuresDir); end

%% ------------------------------------------------------------------
%  PHYSICAL PARAMETERS (illustrative values, in meters / kilograms)
%  ------------------------------------------------------------------
% Chassis (cart) dimensions
cartLength   = 0.40;   % length of chassis along X axis [m]
cartWidth    = 0.30;   % width of chassis along Y axis [m]
cartHeight   = 0.12;   % height of chassis along Z axis [m]

% Wheel dimensions
wheelRadius  = 0.08;   % wheel radius [m]
wheelWidth   = 0.04;   % wheel thickness along Y axis [m]

% Wheel placement (distance between wheel centers)
wheelBaseX   = 0.32;   % distance between front and rear axles [m]
trackWidthY  = cartWidth + wheelWidth + 0.04; % distance between left/right wheel centers [m]

% Pendulum parameters
pendulumLength   = 0.50;   % length of pendulum rod [m]
pendulumMass     = 0.50;   % lumped mass represented by CoM marker [kg]
pendulumCoMFrac  = 0.75;   % fraction of pendulum length where CoM is located

% Derived heights
wheelCenterZ   = wheelRadius;                 % wheel axle height above ground
chassisBottomZ = 2 * wheelRadius;             % chassis sits on top of wheels
chassisTopZ    = chassisBottomZ + cartHeight; % top surface of chassis
pendulumBaseZ  = chassisTopZ;                 % pendulum pivot location
pendulumTopZ   = pendulumBaseZ + pendulumLength;
pendulumCoMZ   = pendulumBaseZ + pendulumCoMFrac * pendulumLength;

fprintf('Four-Wheeled Inverted Pendulum - Physical Parameters\n');
fprintf('-----------------------------------------------------\n');
fprintf('Cart Length x Width x Height : %.2f x %.2f x %.2f m\n', cartLength, cartWidth, cartHeight);
fprintf('Wheel Radius / Width         : %.2f / %.2f m\n', wheelRadius, wheelWidth);
fprintf('Wheelbase (X) / Track (Y)    : %.2f / %.2f m\n', wheelBaseX, trackWidthY);
fprintf('Pendulum Length              : %.2f m\n', pendulumLength);
fprintf('Pendulum Mass                : %.2f kg\n', pendulumMass);
fprintf('Pendulum CoM height above pivot : %.2f m\n', pendulumCoMFrac*pendulumLength);

%% ------------------------------------------------------------------
%  FIGURE SETUP
%  Larger canvas + reserved top margin so the title is never clipped.
%  ------------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [80 60 1100 900]);
ax = axes(fig, 'Position', [0.09 0.08 0.84 0.78]); % leave headroom above for title
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
camlight('headlight');
lighting gouraud;

%% ------------------------------------------------------------------
%  GROUND PLANE
%  ------------------------------------------------------------------
groundSize = 1.3; % half-extent of the ground plane in meters
[gx, gy] = meshgrid(linspace(-groundSize, groundSize, 2), linspace(-groundSize, groundSize, 2));
gz = zeros(size(gx));
surf(ax, gx, gy, gz, 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.9);

%% ------------------------------------------------------------------
%  WHEELS (four cylinders, axis aligned with Y)
%  ------------------------------------------------------------------
wheelPositionsX = [ wheelBaseX/2,  wheelBaseX/2, -wheelBaseX/2, -wheelBaseX/2];
wheelPositionsY = [ trackWidthY/2, -trackWidthY/2, trackWidthY/2, -trackWidthY/2];
wheelNames = {'Front-Left Wheel','Front-Right Wheel','Rear-Left Wheel','Rear-Right Wheel'};

% Unit cylinder template (radius 1, height 1), axis along local Z
[cx, cy, cz] = cylinder(1, 30);

for i = 1:4
    % Scale to actual wheel radius and width
    wx = cx * wheelRadius;
    wy = cy * wheelRadius;
    wz = (cz - 0.5) * wheelWidth; % center the wheel width on its axis

    % Rotate cylinder so its axis points along Y instead of Z
    Xw = wx + wheelPositionsX(i);
    Yw = wz + wheelPositionsY(i);
    Zw = wy + wheelCenterZ;

    surf(ax, Xw, Yw, Zw, 'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none');

    % Cap the two flat sides of each wheel
    thetaCap = linspace(0, 2*pi, 30);
    capX = wheelRadius * cos(thetaCap) + wheelPositionsX(i);
    capZ = wheelRadius * sin(thetaCap) + wheelCenterZ;
    patch(ax, capX, repmat(wheelPositionsY(i) - wheelWidth/2, size(capX)), capZ, [0.15 0.15 0.15], 'EdgeColor', 'none');
    patch(ax, capX, repmat(wheelPositionsY(i) + wheelWidth/2, size(capX)), capZ, [0.15 0.15 0.15], 'EdgeColor', 'none');
end

%% ------------------------------------------------------------------
%  CHASSIS (rectangular box using patch)
%  Slight transparency so all four wheels stay visible beneath it.
%  ------------------------------------------------------------------
cx0 = cartLength/2; cy0 = cartWidth/2;
chassisVertices = [
    -cx0 -cy0 chassisBottomZ;
     cx0 -cy0 chassisBottomZ;
     cx0  cy0 chassisBottomZ;
    -cx0  cy0 chassisBottomZ;
    -cx0 -cy0 chassisTopZ;
     cx0 -cy0 chassisTopZ;
     cx0  cy0 chassisTopZ;
    -cx0  cy0 chassisTopZ ];

chassisFaces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8 ];

patch(ax, 'Vertices', chassisVertices, 'Faces', chassisFaces, ...
    'FaceColor', [0.2 0.45 0.75], 'FaceAlpha', 0.85, 'EdgeColor', 'k', 'LineWidth', 1.2);

%% ------------------------------------------------------------------
%  PENDULUM (rigid rod mounted at chassis center) + CENTER OF MASS
%  ------------------------------------------------------------------
pendulumBaseX = 0; pendulumBaseY = 0;

plot3(ax, [pendulumBaseX pendulumBaseX], [pendulumBaseY pendulumBaseY], ...
    [pendulumBaseZ pendulumTopZ], 'Color', [0.6 0.1 0.1], 'LineWidth', 6);

% Pivot marker at the base of the pendulum
plot3(ax, pendulumBaseX, pendulumBaseY, pendulumBaseZ, 'ko', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 9);

% Center of mass marker (sphere)
[sx, sy, sz] = sphere(20);
comRadius = 0.04;
surf(ax, sx*comRadius + pendulumBaseX, sy*comRadius + pendulumBaseY, ...
    sz*comRadius + pendulumCoMZ, 'FaceColor', [0.9 0.7 0.1], 'EdgeColor', 'none');

%% ------------------------------------------------------------------
%  COORDINATE AXES (for reference)
%  ------------------------------------------------------------------
axisLen = 0.4;
quiver3(ax, 0,0,0, axisLen,0,0, 'r', 'LineWidth', 2.2, 'MaxHeadSize', 0.8);
quiver3(ax, 0,0,0, 0,axisLen,0, 'g', 'LineWidth', 2.2, 'MaxHeadSize', 0.8);
quiver3(ax, 0,0,0, 0,0,axisLen, 'b', 'LineWidth', 2.2, 'MaxHeadSize', 0.8);
text(ax, axisLen*1.12, 0, 0, 'X', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'r');
text(ax, 0, axisLen*1.12, 0, 'Y', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'g');
text(ax, 0, 0, axisLen*1.12, 'Z', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'b');

%% ------------------------------------------------------------------
%  LABELS AND ANNOTATIONS
%  Positioned with extra clearance and background boxes for contrast
%  so nothing overlaps the geometry or gets clipped.
%  ------------------------------------------------------------------
title(ax, 'Four-Wheeled Inverted Pendulum', 'FontSize', 18, 'FontWeight', 'bold');
xlabel(ax, 'X axis [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax, 'Y axis [m]', 'FontSize', 12, 'FontWeight', 'bold');
zlabel(ax, 'Z axis [m]', 'FontSize', 12, 'FontWeight', 'bold');
set(ax, 'FontSize', 10);

text(ax, pendulumBaseX+0.08, pendulumBaseY, (pendulumBaseZ+pendulumTopZ)/2 + 0.05, ...
    'Pendulum', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.6 0.1 0.1], ...
    'BackgroundColor', 'w', 'Margin', 2, 'EdgeColor', [0.6 0.1 0.1]);
text(ax, pendulumBaseX+0.08, pendulumBaseY, pendulumCoMZ - 0.03, ...
    'Center of Mass', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.55 0.42 0.02], ...
    'BackgroundColor', 'w', 'Margin', 2, 'EdgeColor', [0.55 0.42 0.02]);

% Wheel labels: pushed outward and dropped near ground level so they
% sit clear of the wheel geometry and the chassis, with no overlap.
labelOffsetXY = 1.45;   % outward push relative to wheel position
labelZ = wheelCenterZ - wheelRadius - 0.05; % just below wheel, above ground
for i = 1:4
    text(ax, wheelPositionsX(i)*labelOffsetXY, wheelPositionsY(i)*labelOffsetXY, labelZ, ...
        wheelNames{i}, 'FontSize', 9, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'Margin', 1);
end

%% ------------------------------------------------------------------
%  CAMERA / VIEW
%  Oblique angle chosen so the chassis and all four wheels are
%  simultaneously visible and clearly separated.
%  ------------------------------------------------------------------
view(ax, 128, 26);
camva(ax, 8);

%% ------------------------------------------------------------------
%  AXIS LIMITS - generous empty space around the robot
%  ------------------------------------------------------------------
xlim(ax, [-0.95 0.95]);
ylim(ax, [-0.95 0.95]);
zlim(ax, [0 pendulumTopZ + 0.15]);
set(ax, 'DataAspectRatio', [1 1 1]);
box(ax, 'on');

%% ------------------------------------------------------------------
%  SAVE FIGURE FOR README
%  ------------------------------------------------------------------
outFile = fullfile(figuresDir, '01_physical_model.png');
exportgraphics(fig, outFile, 'Resolution', 220);
fprintf('Figure saved to: %s\n', outFile);
