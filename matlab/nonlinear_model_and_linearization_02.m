%% Four-Wheeled Inverted Pendulum - Mathematical Model
% STEP 2 of the robotics portfolio project.
%
% Pipeline implemented in this script:
%   PHYSICAL PARAMETERS -> NONLINEAR EQUATIONS -> LINEARIZATION -> STATE-SPACE MODEL
%
% The four-wheeled chassis is treated as a single equivalent rigid mobile
% base (an equivalent cart) for the purposes of this dynamic model, i.e.
% a classic cart-pole / inverted-pendulum formulation. The four-wheel
% geometry from Step 1 informs the illustrative mass value below but the
% wheels themselves are not modeled individually here.
%
% NOTE: No controller (PID/LQR), no Simulink model, and no disturbance
% simulation are implemented in this step. This is the plant model only.

clear; clc;

%% ------------------------------------------------------------------
%  1) PHYSICAL PARAMETERS
%  ------------------------------------------------------------------
% M : equivalent mass of the four-wheeled chassis/base [kg]
%     (lumped mass representing the cart, wheels, and motors as a
%      single rigid body moving horizontally)
% m : mass of the inverted pendulum [kg]
%     (the rod/pole balanced on top of the chassis)
% l : distance from the pendulum pivot to its center of mass [m]
%     (for a uniform rod pivoted at one end, l = half the rod length)
% I : moment of inertia of the pendulum about its own center of mass [kg*m^2]
%     (resistance of the pendulum to angular acceleration about its CoM)
% g : gravitational acceleration [m/s^2]

M = 2.50;      % equivalent chassis/base mass [kg]
m = 0.50;      % pendulum mass [kg]
l = 0.375;     % pivot-to-CoM distance [m] (matches Step 1: 0.75 * 0.50 m rod)
I = 0.015;     % pendulum moment of inertia about its own CoM [kg*m^2]
g = 9.81;      % gravitational acceleration [m/s^2]

fprintf('=========================================================\n');
fprintf(' STEP 2: MATHEMATICAL MODEL - PHYSICAL PARAMETERS\n');
fprintf('=========================================================\n');
fprintf('M (chassis/base mass)            = %.3f kg\n', M);
fprintf('m (pendulum mass)                = %.3f kg\n', m);
fprintf('l (pivot-to-CoM distance)        = %.3f m\n', l);
fprintf('I (pendulum inertia about CoM)   = %.4f kg*m^2\n', I);
fprintf('g (gravitational acceleration)   = %.2f m/s^2\n\n', g);

%% ------------------------------------------------------------------
%  2) STATE VARIABLES (for reference)
%  ------------------------------------------------------------------
% x         : horizontal displacement of the chassis [m]
% x_dot     : horizontal velocity of the chassis [m/s]
% theta     : pendulum angle measured from the upright vertical [rad]
% theta_dot : pendulum angular velocity [rad/s]
% F         : horizontal control/input force applied to the chassis [N]

%% ------------------------------------------------------------------
%  3) NONLINEAR EQUATIONS OF MOTION (symbolic)
%  ------------------------------------------------------------------
syms x_ddot theta_ddot theta theta_dot F_sym M_sym m_sym l_sym I_sym g_sym real

eq1 = M_sym*x_ddot + m_sym*l_sym*theta_ddot*cos(theta) ...
    - m_sym*l_sym*theta_dot^2*sin(theta) == F_sym;

eq2 = (I_sym + m_sym*l_sym^2)*theta_ddot + m_sym*l_sym*x_ddot*cos(theta) ...
    - m_sym*g_sym*l_sym*sin(theta) == 0;

fprintf('---------------------------------------------------------\n');
fprintf(' NONLINEAR EQUATIONS OF MOTION\n');
fprintf('---------------------------------------------------------\n');
fprintf('Eq (1) - Chassis (horizontal force balance):\n');
disp(eq1);
fprintf('Eq (2) - Pendulum (moment balance about pivot):\n');
disp(eq2);
fprintf('\n');

%% ------------------------------------------------------------------
%  4) SOLVE EXPLICITLY FOR x_ddot AND theta_ddot
%  ------------------------------------------------------------------
solTemp = solve([eq1, eq2], [x_ddot, theta_ddot]);
x_ddot_expr     = simplify(solTemp.x_ddot);
theta_ddot_expr = simplify(solTemp.theta_ddot);

fprintf('---------------------------------------------------------\n');
fprintf(' DERIVED EXPLICIT EXPRESSIONS\n');
fprintf('---------------------------------------------------------\n');
fprintf('x_ddot =\n'); disp(x_ddot_expr);
fprintf('theta_ddot =\n'); disp(theta_ddot_expr);
fprintf('\n');

%% ------------------------------------------------------------------
%  5) LINEARIZATION ABOUT THE UPRIGHT EQUILIBRIUM
%     (theta = 0, theta_dot = 0, x_dot = 0)
%  ------------------------------------------------------------------
% Introduce explicit state symbols so the Jacobian is taken with
% respect to the actual state vector X = [x; x_dot; theta; theta_dot].
syms x1 x2 real   % x1 = x (position), x2 = x_dot (velocity)
% theta and theta_dot are already symbolic states (x3, x4 conceptually)

% Nonlinear state-derivative vector f(X,F) = X_dot
f_vec = [ x2;
          x_ddot_expr;
          theta_dot;
          theta_ddot_expr ];

stateVec = [x1; x2; theta; theta_dot];

A_sym = jacobian(f_vec, stateVec);
B_sym = jacobian(f_vec, F_sym);

% Evaluate the Jacobians at the upright equilibrium point
A_lin = subs(A_sym, [theta, theta_dot], [0, 0]);
B_lin = subs(B_sym, [theta, theta_dot], [0, 0]);
A_lin = simplify(A_lin);
B_lin = simplify(B_lin);

fprintf('---------------------------------------------------------\n');
fprintf(' LINEARIZED STATE-SPACE MODEL (symbolic, about theta=0)\n');
fprintf('---------------------------------------------------------\n');
fprintf('State vector: X = [x; x_dot; theta; theta_dot]\n');
fprintf('  x         - chassis horizontal displacement [m]\n');
fprintf('  x_dot     - chassis horizontal velocity [m/s]\n');
fprintf('  theta     - pendulum angle from upright [rad]\n');
fprintf('  theta_dot - pendulum angular velocity [rad/s]\n');
fprintf('Model: X_dot = A*X + B*F\n\n');
fprintf('Symbolic A matrix:\n'); disp(A_lin);
fprintf('Symbolic B matrix:\n'); disp(B_lin);
fprintf('\n');

%% ------------------------------------------------------------------
%  6) NUMERIC A AND B MATRICES (substitute physical parameter values)
%  ------------------------------------------------------------------
A = double(subs(A_lin, [M_sym, m_sym, l_sym, I_sym, g_sym], [M, m, l, I, g]));
B = double(subs(B_lin, [M_sym, m_sym, l_sym, I_sym, g_sym], [M, m, l, I, g]));

fprintf('=========================================================\n');
fprintf(' NUMERIC LINEAR STATE-SPACE MODEL\n');
fprintf('=========================================================\n');
fprintf('A matrix (4x4):\n');
disp(A);
fprintf('B matrix (4x1):\n');
disp(B);

%% ------------------------------------------------------------------
%  7) DIMENSION VERIFICATION
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' DIMENSION CHECKS\n');
fprintf('---------------------------------------------------------\n');
[nA_rows, nA_cols] = size(A);
[nB_rows, nB_cols] = size(B);
fprintf('size(A) = [%d x %d]  (expected 4 x 4)\n', nA_rows, nA_cols);
fprintf('size(B) = [%d x %d]  (expected 4 x 1)\n', nB_rows, nB_cols);

assert(isequal(size(A), [4 4]), 'A matrix does not have expected dimensions 4x4.');
assert(isequal(size(B), [4 1]), 'B matrix does not have expected dimensions 4x1.');
fprintf('PASS: A and B have the expected state-space dimensions.\n\n');

%% ------------------------------------------------------------------
%  8) BASIC NUMERICAL SANITY CHECKS
%  ------------------------------------------------------------------
fprintf('---------------------------------------------------------\n');
fprintf(' NUMERICAL SANITY CHECKS\n');
fprintf('---------------------------------------------------------\n');

% Check 1: kinematic identities x_dot = x_dot and theta_dot = theta_dot
% should appear directly as 1 entries in A (rows 1 and 3).
tol = 1e-9;
check1 = abs(A(1,2) - 1) < tol && abs(A(3,4) - 1) < tol;
fprintf('Check 1 (kinematic identities A(1,2)=1, A(3,4)=1): %s\n', mat2str(check1));

% Check 2: rows for x and theta (positions) should have zero derivative
% dependence on the states themselves at equilibrium (no restoring force
% on x, and the position row only depends on the velocity state).
check2 = all(abs(A(1,:) - [0 1 0 0]) < tol);
fprintf('Check 2 (row 1 of A equals [0 1 0 0]):              %s\n', mat2str(check2));

% Check 3: the (theta, theta) entry of A should be ZERO (gravity torque
% is linear in theta only through the coefficient in column 3, row 4,
% not row 3), confirming theta_dot row is purely kinematic.
check3 = all(abs(A(3,:) - [0 0 0 1]) < tol);
fprintf('Check 3 (row 3 of A equals [0 0 0 1]):              %s\n', mat2str(check3));

% Check 4: physical instability of the inverted pendulum. An upright
% pendulum is an unstable equilibrium, so the linearized A matrix must
% have at least one eigenvalue with strictly positive real part.
eigA = eig(A);
hasUnstablePole = any(real(eigA) > tol);
fprintf('Check 4 (A has an unstable pole, as physically expected): %s\n', mat2str(hasUnstablePole));
fprintf('   Eigenvalues of A:\n');
for k = 1:numel(eigA)
    fprintf('     lambda_%d = %s\n', k, num2str(eigA(k)));
end

% Check 5: B should have zero entries in the position/angle rows (a
% force cannot instantaneously change position or angle, only rates).
check5 = abs(B(1)) < tol && abs(B(3)) < tol;
fprintf('Check 5 (B(1)=0 and B(3)=0, force affects rates only): %s\n', mat2str(check5));

% Check 6: sign check on the theta_ddot equation - for a small positive
% tilt (theta > 0) with zero input force, gravity should accelerate the
% pendulum further away from vertical (positive feedback / instability).
thetaSmall = 0.05; % rad, small perturbation
thetaddot_check = double(subs(theta_ddot_expr, ...
    [M_sym, m_sym, l_sym, I_sym, g_sym, theta, theta_dot, F_sym], ...
    [M, m, l, I, g, thetaSmall, 0, 0]));
check6 = thetaddot_check > 0;
fprintf('Check 6 (small tilt grows under gravity, F=0):      %s (theta_ddot = %.4f rad/s^2)\n', ...
    mat2str(check6), thetaddot_check);

allChecksPassed = check1 && check2 && check3 && hasUnstablePole && check5 && check6;
fprintf('\n');
if allChecksPassed
    fprintf('ALL SANITY CHECKS PASSED.\n');
else
    fprintf('WARNING: One or more sanity checks failed. Review the model.\n');
end

fprintf('\n=========================================================\n');
fprintf(' STEP 2 COMPLETE: Nonlinear model derived and linearized.\n');
fprintf(' No controller, no Simulink, no disturbance simulation.\n');
fprintf('=========================================================\n');
