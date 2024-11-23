% derive_equations.m

% Clear workspace and command window
clear all;
close all;
clc;

% Define symbolic variables
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real % Generalized coordinates and their derivatives
syms L1 L2 L3 m1 m2 m3 g real % System parameters

% Define positions of centers of mass
x1 = (L1/2)*sin(q1);
y1 = -(L1/2)*cos(q1);

x2 = L1*sin(q1) + (L2/2)*sin(q1 + q2);
y2 = -L1*cos(q1) - (L2/2)*cos(q1 + q2);

x3 = L1*sin(q1) + L2*sin(q1 + q2) + (L3/2)*sin(q1 + q2 + q3);
y3 = -L1*cos(q1) - L2*cos(q1 + q2) - (L3/2)*cos(q1 + q2 + q3);

% Compute velocities of centers of mass
vx1 = diff(x1, q1)*dq1;
vy1 = diff(y1, q1)*dq1;

vx2 = diff(x2, q1)*dq1 + diff(x2, q2)*dq2;
vy2 = diff(y2, q1)*dq1 + diff(y2, q2)*dq2;

vx3 = diff(x3, q1)*dq1 + diff(x3, q2)*dq2 + diff(x3, q3)*dq3;
vy3 = diff(y3, q1)*dq1 + diff(y3, q2)*dq2 + diff(y3, q3)*dq3;

% Compute kinetic energy
T = (1/2)*m1*(vx1^2 + vy1^2) + (1/2)*m2*(vx2^2 + vy2^2) + (1/2)*m3*(vx3^2 + vy3^2);

% Compute potential energy
V = m1*g*y1 + m2*g*y2 + m3*g*y3;

% Compute Lagrangian
L = T - V;

% Compute partial derivatives of Lagrangian with respect to q and dq
dL_dq1 = diff(L, q1);
dL_dq2 = diff(L, q2);
dL_dq3 = diff(L, q3);

dL_ddq1 = diff(L, dq1);
dL_ddq2 = diff(L, dq2);
dL_ddq3 = diff(L, dq3);

% Compute time derivatives of partial derivatives (chain rule)
% Since q1, q2, q3 are functions of time, their total derivatives involve derivatives of q and dq
syms t real
d_dt_dL_ddq1 = diff(dL_ddq1, q1)*dq1 + diff(dL_ddq1, q2)*dq2 + diff(dL_ddq1, q3)*dq3 + ...
               diff(dL_ddq1, dq1)*ddq1 + diff(dL_ddq1, dq2)*ddq2 + diff(dL_ddq1, dq3)*ddq3;

d_dt_dL_ddq2 = diff(dL_ddq2, q1)*dq1 + diff(dL_ddq2, q2)*dq2 + diff(dL_ddq2, q3)*dq3 + ...
               diff(dL_ddq2, dq1)*ddq1 + diff(dL_ddq2, dq2)*ddq2 + diff(dL_ddq2, dq3)*ddq3;

d_dt_dL_ddq3 = diff(dL_ddq3, q1)*dq1 + diff(dL_ddq3, q2)*dq2 + diff(dL_ddq3, q3)*dq3 + ...
               diff(dL_ddq3, dq1)*ddq1 + diff(dL_ddq3, dq2)*ddq2 + diff(dL_ddq3, dq3)*ddq3;

% Compute equations of motion
eq1 = simplify(d_dt_dL_ddq1 - dL_dq1);
eq2 = simplify(d_dt_dL_ddq2 - dL_dq2);
eq3 = simplify(d_dt_dL_ddq3 - dL_dq3);

% Solve for angular accelerations (ddq1, ddq2, ddq3)
[sol_ddq1, sol_ddq2, sol_ddq3] = solve([eq1 == 0, eq2 == 0, eq3 == 0], [ddq1, ddq2, ddq3]);

% Simplify solutions
sol_ddq1 = simplify(sol_ddq1);
sol_ddq2 = simplify(sol_ddq2);
sol_ddq3 = simplify(sol_ddq3);

% Convert symbolic expressions to MATLAB functions
matlabFunction(sol_ddq1, 'File', 'compute_ddq1', 'Vars', {q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g});
matlabFunction(sol_ddq2, 'File', 'compute_ddq2', 'Vars', {q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g});
matlabFunction(sol_ddq3, 'File', 'compute_ddq3', 'Vars', {q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g});
