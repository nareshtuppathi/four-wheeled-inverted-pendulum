%% Four-Wheeled Inverted Pendulum - LQR Controller Design
% STEP 3 of the robotics portfolio project.
%
% Pipeline implemented in this script:
%   STATE-SPACE MODEL -> CONTROLLABILITY -> LQR DESIGN -> GAIN K -> CLOSED-LOOP STABILITY
%
% This script ONLY designs a static full-state-feedback LQR gain K for
% the linearized plant derived in Step 2 (four_wheeled_inverted_pendulum_model.m).
% No time-domain simulation, no disturbance rejection, and no Simulink
% model are included here.

clear; clc;

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS (identical to Step 2)
%  ------------------------------------------------------------------
% M : equivalent chassis/base mass [kg]
% m : pendulum mass [kg]
% l : pivot-to-CoM distance [m]
% I : pendulum moment of inertia about its own CoM [kg*m^2]
% g : gravitational acceleration [m/s^2]
M = 2.50;
m = 0.50;
l = 0.375;
I = 0.015;
g = 9.81;

fprintf('=========================================================\n');
fprintf(' STEP 3: LQR CONTROLLER DESIGN\n');
fprintf('=========================================================\n');
fprintf('Physical parameters: M=%.2f kg, m=%.2f kg, l=%.3f m, I=%.4f kg*m^2, g=%.2f m/s^2\n\n', ...
    M, m, l, I, g);

%% ------------------------------------------------------------------
%  2) LINEARIZED STATE-SPACE MODEL (reconstructed from Step 2)
%     State vector: X = [x; x_dot; theta; theta_dot]
%       x         - chassis horizontal displacement [m]
%       x_dot     - chassis horizontal velocity [m/s]
%       theta     - pendulum angle from upright [rad]
%       theta_dot - pendulum angular velocity [rad/s]
%     Input: F - horizontal control force on the chassis [N]
%     Model: X_dot = A*X + B*F
%  ------------------------------------------------------------------
D = M*(I + m*l^2) - (m*l)^2;   % common denominator from the Step 2 derivation

A = [ 0                       1   0                              0;
      0                       0   -(g*m^2*l^2)/D                 0;
      0                       0   0                              1;
      0                       0   (M*g*m*l)/D                    0 ];

B = [ 0;
      (m*l^2 + I)/D;
      0;
      -(m*l)/D ];

fprintf('---------------------------------------------------------\n');
fprintf(' STATE-SPACE MODEL (from Step 2)\n');
fprintf('---------------------------------------------------------\n');
fprintf('A matrix:\n'); disp(A);
fprintf('B matrix:\n'); disp(B);

%% ------------------------------------------------------------------
%  3) CONTROLLABILITY CHECK
%  ------------------------------------------------------------------
% A linear system is controllable if the controllability matrix
% Co = [B, A*B, A^2*B, A^3*B] has full rank (equal to the number of
% states). Controllability must be confirmed before LQR design makes
% sense, since LQR relies on being able to place closed-loop poles
% via full-state feedback.
Co = ctrb(A, B);
rankCo = rank(Co);
nStates = size(A, 1);

fprintf('\n---------------------------------------------------------\n');
fprintf(' CONTROLLABILITY CHECK\n');
fprintf('---------------------------------------------------------\n');
fprintf('Controllability matrix Co = ctrb(A,B):\n'); disp(Co);
fprintf('rank(Co) = %d   (number of states = %d)\n', rankCo, nStates);

isControllable = (rankCo == nStates);
if isControllable
    fprintf('RESULT: The system IS controllable (full rank). LQR design can proceed.\n\n');
else
    error('The system is NOT controllable (rank(Co) = %d < %d). Cannot proceed with LQR.', ...
        rankCo, nStates);
end

%% ------------------------------------------------------------------
%  4) LQR WEIGHTING MATRICES Q AND R
%  ------------------------------------------------------------------
% Q penalizes state deviations; R penalizes control effort. Diagonal Q
% is used since we care about each state independently, with weights
% chosen as follows:
%
%   Q(1,1) = 1   -> x (chassis position): least critical. Some drift in
%                   chassis position is acceptable as long as the
%                   pendulum stays upright, so it gets the lowest weight.
%   Q(2,2) = 1   -> x_dot (chassis velocity): similarly low priority,
%                   kept equal to position so the base is not penalized
%                   more than necessary while the pendulum is balanced.
%   Q(3,3) = 100 -> theta (pendulum angle): HIGHEST priority. This is
%                   the whole point of the controller - keep the
%                   pendulum upright - so deviations in angle are
%                   penalized far more heavily than position errors.
%   Q(4,4) = 10  -> theta_dot (pendulum angular velocity): second
%                   highest priority. Damping the angular rate quickly
%                   prevents the pendulum from swinging past the point
%                   of recoverability.
%
%   R = 1        -> a moderate penalty on control force F. This value
%                   trades off aggressive correction against actuator
%                   effort; smaller R would produce a more aggressive
%                   (higher-gain) controller, larger R a gentler one.
Q = diag([1, 1, 100, 10]);
R = 1;

fprintf('---------------------------------------------------------\n');
fprintf(' LQR WEIGHTING MATRICES\n');
fprintf('---------------------------------------------------------\n');
fprintf('Q (state weighting matrix):\n'); disp(Q);
fprintf('R (control effort weighting):\n'); disp(R);

%% ------------------------------------------------------------------
%  5) LQR GAIN CALCULATION
%     Control law: F = -K*X
%  ------------------------------------------------------------------
[K, S, openLoopPolesFromLQR] = lqr(A, B, Q, R);

fprintf('\n---------------------------------------------------------\n');
fprintf(' LQR GAIN MATRIX K\n');
fprintf('---------------------------------------------------------\n');
fprintf('Control law: F = -K*X\n');
fprintf('K = [Kx, Kx_dot, Ktheta, Ktheta_dot] =\n'); disp(K);

%% ------------------------------------------------------------------
%  6) OPEN-LOOP EIGENVALUES (uncontrolled plant, from Step 2)
%  ------------------------------------------------------------------
openLoopEig = eig(A);
fprintf('---------------------------------------------------------\n');
fprintf(' OPEN-LOOP EIGENVALUES (of A, no control)\n');
fprintf('---------------------------------------------------------\n');
for k = 1:numel(openLoopEig)
    fprintf('  lambda_%d = %s\n', k, num2str(openLoopEig(k)));
end
fprintf('(Expect at least one eigenvalue with positive real part -\n');
fprintf(' the open-loop pendulum is unstable, as confirmed in Step 2.)\n\n');

%% ------------------------------------------------------------------
%  7) CLOSED-LOOP SYSTEM AND EIGENVALUES
%  ------------------------------------------------------------------
A_cl = A - B*K;
closedLoopEig = eig(A_cl);

fprintf('---------------------------------------------------------\n');
fprintf(' CLOSED-LOOP MATRIX A_cl = A - B*K\n');
fprintf('---------------------------------------------------------\n');
disp(A_cl);

fprintf('---------------------------------------------------------\n');
fprintf(' CLOSED-LOOP EIGENVALUES (of A_cl = A - B*K)\n');
fprintf('---------------------------------------------------------\n');
for k = 1:numel(closedLoopEig)
    fprintf('  lambda_%d = %s\n', k, num2str(closedLoopEig(k)));
end
fprintf('\n');

%% ------------------------------------------------------------------
%  8) STABILITY VERIFICATION
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' STABILITY VERIFICATION\n');
fprintf('---------------------------------------------------------\n');
tol = 1e-9;
maxRealPart = max(real(closedLoopEig));
isStable = all(real(closedLoopEig) < -tol);

fprintf('Maximum real part among closed-loop eigenvalues: %.6f\n', maxRealPart);
fprintf('All closed-loop eigenvalues have negative real parts: %s\n', mat2str(isStable));

assert(isStable, 'Closed-loop system is NOT stable - check Q, R, or model.');
fprintf('\nRESULT: Closed-loop system IS STABLE under LQR feedback F = -K*X.\n');

%% ------------------------------------------------------------------
%  9) ADDITIONAL NUMERICAL SANITY CHECKS
%  ------------------------------------------------------------------
fprintf('\n---------------------------------------------------------\n');
fprintf(' ADDITIONAL SANITY CHECKS\n');
fprintf('---------------------------------------------------------\n');

% Check 1: K should have 1 row (single input F) and 4 columns (4 states)
check1 = isequal(size(K), [1, 4]);
fprintf('Check 1 (size(K) == [1 4]):                         %s\n', mat2str(check1));

% Check 2: gain on theta should be the largest in magnitude, reflecting
% the heavy Q weighting placed on the pendulum angle.
[~, idxMax] = max(abs(K));
check2 = (idxMax == 3);
fprintf('Check 2 (largest gain magnitude is on theta, col 3): %s\n', mat2str(check2));

% Check 3: closed-loop poles should differ from open-loop poles (i.e.
% feedback actually changed the dynamics).
check3 = ~isequal(sort(openLoopEig), sort(closedLoopEig));
fprintf('Check 3 (closed-loop poles differ from open-loop):  %s\n', mat2str(check3));

% Check 4: closed-loop system should be controllable-consistent, i.e.
% A_cl should not itself be singular in a way that implies a hidden
% uncontrollable unstable mode (rank of Co already confirmed full).
check4 = isControllable;
fprintf('Check 4 (underlying open-loop system controllable):  %s\n', mat2str(check4));

allChecksPassed = check1 && check2 && check3 && check4 && isStable;
fprintf('\n');
if allChecksPassed
    fprintf('ALL SANITY CHECKS PASSED.\n');
else
    fprintf('WARNING: One or more sanity checks failed. Review the design.\n');
end

fprintf('\n=========================================================\n');
fprintf(' STEP 3 COMPLETE: LQR gain K designed, closed-loop stability confirmed.\n');
fprintf(' No time-domain simulation, disturbances, or Simulink model added.\n');
fprintf('=========================================================\n');
