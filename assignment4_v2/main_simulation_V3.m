% main_simulation.m

% Clear workspace and command window
clear all;
close all;
clc;

% Define parameters
params.L1 = 0.5; params.L2 = 0.5; params.L3 = 0.25;
params.m1 = 0.5; params.m2 = 0.5; params.m3 = 0.25;
params.g = 9.81;

% Initial conditions (angles in radians, velocities in rad/s)
q1_0 = deg2rad(10);
q2_0 = deg2rad(40);
q3_0 = deg2rad(40);
dq1_0 = 0;
dq2_0 = 0;
dq3_0 = 0;
y0 = [q1_0; q2_0; q3_0; dq1_0; dq2_0; dq3_0];

% Time span for simulation
tspan = [0 10];

% Solve the ODE using ode45
[t, y] = ode45(@(t, y) equations_of_motion(t, y, params), tspan, y0);

% Extract joint angles and velocities
q1 = y(:,1); q2 = y(:,2); q3 = y(:,3);
dq1 = y(:,4); dq2 = y(:,5); dq3 = y(:,6);

% Create variables for theta1, theta2, theta3 (angles over time)
theta1 = q1; % Just Theta1 in radians
theta2 = q2; % Just Theta2 in radians
theta3 = q3; % Just Theta3 in radians

% Save theta variables and time vector to a .mat file for Simulink
save('theta_data.mat', 't', 'theta1', 'theta2', 'theta3');

% Plot joint angles in degrees
q1_deg = rad2deg(theta1); q2_deg = rad2deg(theta2); q3_deg = rad2deg(theta3);
figure;
plot(t, q1_deg, 'r', 'LineWidth', 1.5); hold on;
plot(t, q2_deg, 'g', 'LineWidth', 1.5);
plot(t, q3_deg, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (degrees)');
legend('\theta_1', '\theta_2', '\theta_3');
title('Joint Angles vs Time');
grid on;

% Plot joint velocities in rad/s
figure;
plot(t, dq1, 'r', 'LineWidth', 1.5); hold on;
plot(t, dq2, 'g', 'LineWidth', 1.5);
plot(t, dq3, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('\omega_1', '\omega_2', '\omega_3');
title('Joint Velocities vs Time');
grid on;
