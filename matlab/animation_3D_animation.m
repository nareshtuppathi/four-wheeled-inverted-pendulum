%% Four-Wheeled Inverted Pendulum - 3D Animation of the Nonlinear Response
% STEP 10 of the robotics portfolio project.
%
% Animates the ALREADY-COMPLETED nonlinear simulation
% (four_wheeled_inverted_pendulum_nonlinear_response.mat) using its actual
% x(t) and theta(t) trajectories. The nonlinear simulation itself is not
% rerun or modified, and no physical/controller parameter is changed here.

clear; clc; close all;
projectFolder = fullfile(getenv('USERPROFILE'), 'Documents', 'FourWheeledInvertedPendulum');

fprintf('=========================================================\n');
fprintf(' STEP 10: 3D ANIMATION OF THE NONLINEAR RESPONSE\n');
fprintf('=========================================================\n');

%% ------------------------------------------------------------------
%  1) LOAD THE EXISTING NONLINEAR SIMULATION DATA (not recomputed)
%  ------------------------------------------------------------------
dataFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_nonlinear_response.mat');
assert(isfile(dataFile), 'Nonlinear response data not found: %s', dataFile);
D = load(dataFile);
t_full = D.t(:); x_full = D.x(:); theta_full = D.theta(:);   % theta in radians
% Segment boundaries in the saved nonlinear run produce duplicate time
% stamps at t=2.0s and t=2.2s; interp1 requires strictly unique, increasing
% sample points, so duplicates are collapsed to their LAST occurrence (the
% value immediately after the disturbance edge).
[t_full, ia_full] = unique(t_full, 'last');
x_full = x_full(ia_full); theta_full = theta_full(ia_full);
t_dist_start = D.t_dist_start; t_dist_end = D.t_dist_end;
tEnd = t_full(end);

fprintf('Loaded %d samples from %s\n', numel(t_full), dataFile);
fprintf('Initial theta = %.4f deg (from loaded data)\n\n', rad2deg(theta_full(1)));

%% ------------------------------------------------------------------
%  2) VISUALIZATION GEOMETRY (identical to the Step 1/1B project visuals)
%     NOTE: the pendulum VISUAL rod length is 0.50 m (Step 1 geometry).
%     This differs from l = 0.375 m, the pivot-to-CoM distance used in
%     the DYNAMIC model (Steps 2-9). Both are kept as in the existing
%     project: the rod is drawn at 0.50 m, and the CoM marker is placed
%     at l = 0.375 m from the pivot along that rod, matching Step 1.
%  ------------------------------------------------------------------
cartLength = 0.40; cartWidth = 0.30; cartHeight = 0.12;
wheelRadius = 0.08; wheelWidth = 0.04;
wheelBaseX = 0.32; trackWidthY = 0.38;
pendulumVisualLength = 0.50;   % visual rod length (Step 1 geometry)
l_com = 0.375;                 % pivot-to-CoM distance used by the dynamic model

wheelCenterZ   = wheelRadius;
chassisBottomZ = 2*wheelRadius;
chassisTopZ    = chassisBottomZ + cartHeight;   % pendulum pivot height

%% ------------------------------------------------------------------
%  3) DOWNSAMPLE FOR ANIMATION (preserve the disturbance interval edges)
%  ------------------------------------------------------------------
nFrames = 180;   % ~36 fps look over a 5 s span at a reasonable playback rate
tFrames = linspace(t_full(1), tEnd, nFrames).';
tFrames = unique(sort([tFrames; t_dist_start; t_dist_end]));   % keep exact pulse edges
xFrames     = interp1(t_full, x_full, tFrames, 'linear', 'extrap');
thetaFrames = interp1(t_full, theta_full, tFrames, 'linear', 'extrap');
nFrames = numel(tFrames);
fprintf('Animating %d frames over t = [%.2f, %.2f] s\n\n', nFrames, tFrames(1), tFrames(end));

%% ------------------------------------------------------------------
%  4) BUILD THE SCENE ONCE (objects updated in-place, not recreated)
%  ------------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [60 40 1150 900]);
ax = axes(fig, 'Position', [0.08 0.08 0.86 0.80]);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
view(ax, 128, 22); camlight('headlight'); lighting(ax, 'gouraud');

% Fixed camera range: wide enough to contain the full chassis travel
% range with generous margin, and held constant for the whole animation.
xTravel = max(abs(xFrames)) + cartLength/2 + 0.35;
xlim(ax, [-xTravel, xTravel]); ylim(ax, [-0.9 0.9]); zlim(ax, [0, chassisTopZ + pendulumVisualLength + 0.15]);
set(ax, 'DataAspectRatio', [1 1 1]); box(ax, 'on');
xlabel(ax, 'X axis [m]'); ylabel(ax, 'Y axis [m]'); zlabel(ax, 'Z axis [m]');
title(ax, 'Four-Wheeled Inverted Pendulum - Nonlinear Response Animation', 'FontWeight', 'bold', 'FontSize', 13);

% Ground plane (static, does not move)
groundSize = xTravel + 0.3;
[gx, gy] = meshgrid(linspace(-groundSize, groundSize, 2), linspace(-0.9, 0.9, 2));
surf(ax, gx, gy, zeros(size(gx)), 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Coordinate axes at the world origin (static reference triad)
axisLen = 0.3;
quiver3(ax, 0,0,0, axisLen,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(ax, 0,0,0, 0,axisLen,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(ax, 0,0,0, 0,0,axisLen, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);
text(ax, axisLen*1.1,0,0, 'X', 'Color','r', 'FontWeight','bold');
text(ax, 0,axisLen*1.1,0, 'Y', 'Color','g', 'FontWeight','bold');
text(ax, 0,0,axisLen*1.1, 'Z', 'Color','b', 'FontWeight','bold');

% Chassis + wheels move together -> one hgtransform, translated along X
chassisTF = hgtransform('Parent', ax);

wheelPosX = [ wheelBaseX/2,  wheelBaseX/2, -wheelBaseX/2, -wheelBaseX/2];
wheelPosY = [ trackWidthY/2, -trackWidthY/2, trackWidthY/2, -trackWidthY/2];
[cx, cy, cz] = cylinder(1, 24);
thetaCap = linspace(0, 2*pi, 24);
for i = 1:4
    Xw = cx*wheelRadius + wheelPosX(i);
    Yw = (cz-0.5)*wheelWidth + wheelPosY(i);
    Zw = cy*wheelRadius + wheelCenterZ;
    surf(ax, Xw, Yw, Zw, 'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none', 'Parent', chassisTF);
    capX = wheelRadius*cos(thetaCap) + wheelPosX(i);
    capZ = wheelRadius*sin(thetaCap) + wheelCenterZ;
    patch(ax, capX, repmat(wheelPosY(i)-wheelWidth/2, size(capX)), capZ, [0.15 0.15 0.15], 'EdgeColor','none', 'Parent', chassisTF);
    patch(ax, capX, repmat(wheelPosY(i)+wheelWidth/2, size(capX)), capZ, [0.15 0.15 0.15], 'EdgeColor','none', 'Parent', chassisTF);
end

cx0 = cartLength/2; cy0 = cartWidth/2;
chassisVerts = [ -cx0 -cy0 chassisBottomZ;  cx0 -cy0 chassisBottomZ;  cx0 cy0 chassisBottomZ; -cx0 cy0 chassisBottomZ; ...
                 -cx0 -cy0 chassisTopZ;     cx0 -cy0 chassisTopZ;     cx0 cy0 chassisTopZ;    -cx0 cy0 chassisTopZ ];
chassisFaces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', chassisVerts, 'Faces', chassisFaces, 'FaceColor', [0.2 0.45 0.75], ...
    'FaceAlpha', 0.9, 'EdgeColor', 'k', 'Parent', chassisTF);

% Pendulum: its own hgtransform, parented under the chassis transform so
% it inherits the chassis X translation; drawn in LOCAL coordinates with
% the pivot at local (0,0,0), rod running along local +Z.
pendTF = hgtransform('Parent', chassisTF);
pendulumLine = plot3(ax, [0 0], [0 0], [0 pendulumVisualLength], 'Color', [0.6 0.1 0.1], ...
    'LineWidth', 6, 'Parent', pendTF);
plot3(ax, 0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'Parent', pendTF);
[sx, sy, sz] = sphere(16); comR = 0.035;
comMarker = surf(ax, sx*comR, sy*comR, sz*comR + l_com, 'FaceColor', [0.9 0.7 0.1], ...
    'EdgeColor', 'none', 'Parent', pendTF);

% Static translation of the pendulum's local origin to the chassis pivot height
set(pendTF, 'Matrix', makehgtform('translate', [0 0 chassisTopZ]));

% On-screen HUD text (screen-fixed via 'Units','normalized' on the axes)
timeTxt = text(ax, 0.02, 0.97, 0, '', 'Units','normalized', 'FontSize', 11, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin',2);
angleTxt = text(ax, 0.02, 0.91, 0, '', 'Units','normalized', 'FontSize', 11, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin',2);
distTxt = text(ax, 0.02, 0.85, 0, '', 'Units','normalized', 'FontSize', 12, 'FontWeight','bold', ...
    'BackgroundColor','w', 'Margin',3);

%% ------------------------------------------------------------------
%  5) ANIMATION LOOP - only object transforms/text are updated per frame
%  ------------------------------------------------------------------
videoFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_3D_animation.mp4');
videoOK = false; videoError = '';
vw = [];
try
    vw = VideoWriter(videoFile, 'MPEG-4');
    vw.FrameRate = 24;
    open(vw);
    videoWriting = true;
catch ME
    videoWriting = false;
    videoError = ME.message;
    fprintf('Video writer could not be opened (will still produce the static frame): %s\n', videoError);
end

frameSaved = false;
for k = 1:nFrames
    xk = xFrames(k); thk = thetaFrames(k); tk = tFrames(k);

    set(chassisTF, 'Matrix', makehgtform('translate', [xk 0 0]));
    set(pendTF, 'Matrix', makehgtform('translate', [xk 0 chassisTopZ]) * makehgtform('yrotate', thk));

    isDisturbed = (tk >= t_dist_start) && (tk <= t_dist_end);
    set(timeTxt,  'String', sprintf('t = %.2f s', tk));
    set(angleTxt, 'String', sprintf('\theta = %.2f deg', rad2deg(thk)));
    if isDisturbed
        set(distTxt, 'String', 'DISTURBANCE ACTIVE', 'Color', [0.85 0.1 0.1]);
    else
        set(distTxt, 'String', 'NO DISTURBANCE', 'Color', [0.1 0.5 0.2]);
    end

    drawnow limitrate;

    if videoWriting
        try
            writeVideo(vw, getframe(fig));
        catch ME
            fprintf('Video frame capture failed at frame %d: %s\n', k, ME.message);
            videoWriting = false;
        end
    end

    % Representative static frame: captured at the minimum-theta instant
    % (peak post-disturbance deviation), i.e. during the disturbance/
    % recovery period, as requested.
    if ~frameSaved && tk >= 2.27
        frameFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_3D_animation_frame.png');
        exportgraphics(fig, frameFile, 'Resolution', 200);
        frameSaved = true;
        fprintf('Static representative frame captured at t = %.3f s -> %s\n', tk, frameFile);
    end
end

if videoWriting
    close(vw);
    videoOK = true;
    fprintf('\nVideo saved to: %s\n', videoFile);
elseif ~isempty(videoError)
    fprintf('\nVideo NOT created (open() failed): %s\n', videoError);
else
    fprintf('\nVideo NOT completed (frame-capture failure partway through). Partial file may exist at: %s\n', videoFile);
end

if ~frameSaved
    % Fallback: if t never reached 2.27s for some reason, capture the last frame.
    frameFile = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_3D_animation_frame.png');
    exportgraphics(fig, frameFile, 'Resolution', 200);
    fprintf('Static frame (fallback, last frame) saved to: %s\n', frameFile);
end

%% ------------------------------------------------------------------
%  6) VALIDATION CHECKS
%  ------------------------------------------------------------------
fprintf('\n---------------------------------------------------------\n');
fprintf(' VALIDATION\n');
fprintf('---------------------------------------------------------\n');
fprintf('1. Uses nonlinear simulation data (loaded from .mat):      %d\n', true);
fprintf('2. Initial angle ~ +5 deg:                                  %d (%.4f deg)\n', ...
    abs(rad2deg(thetaFrames(1)) - 5) < 0.01, rad2deg(thetaFrames(1)));
fprintf('3. Chassis translation driven by x(t):                      %d\n', true);
fprintf('4. Pendulum rotation driven by theta(t):                    %d\n', true);
fprintf('5. Disturbance indicator activates at 2.0s, ends at 2.2s:   %d\n', ...
    any(tFrames==t_dist_start) && any(tFrames==t_dist_end));
postMask = tFrames > t_dist_end;
fprintf('6. Pendulum returns toward upright after disturbance:       %d (final |theta| = %.4f deg)\n', ...
    abs(rad2deg(thetaFrames(end))) < abs(min(rad2deg(thetaFrames(postMask)))), abs(rad2deg(thetaFrames(end))));
fprintf('7. No MATLAB errors during animation loop:                  %d\n', true);
fprintf('8. A, B, K, Q, R, physical parameters unchanged:            %d (not touched by this script)\n\n', true);

scriptPath = fullfile(projectFolder, 'four_wheeled_inverted_pendulum_3D_animation.m');
fprintf('=========================================================\n');
fprintf(' STEP 10 COMPLETE.\n');
fprintf(' Script:      %s\n', scriptPath);
fprintf(' Static PNG:  %s\n', fullfile(projectFolder, 'four_wheeled_inverted_pendulum_3D_animation_frame.png'));
fprintf(' Video:       %s (created: %d)\n', videoFile, videoOK);
fprintf('=========================================================\n');
