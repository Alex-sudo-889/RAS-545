% FANUC 200iD Forward Kinematics and Visualization

clear all; clc;

% Define symbolic variables for joint angles and link offsets
syms theta1 theta2 theta3 theta4 theta5
syms d1 d2 d3 d4 d5

% Homogeneous Transformation Matrices between each link

% Transformation from frame 0 to frame 1 (T1)
H01 = [cos(theta1),  0, -sin(theta1), 0;  % Correct rotation for frame 0 to frame 1
       sin(theta1),  0,  cos(theta1), 0;  % Axis rotation about Z-axis
                0, -1,           0, d1;   % Link offset along Z-axis
                0,  0,           0, 1];

% Transformation from frame 1 to frame 2 (T2)
H12 = [cos(theta2), -sin(theta2), 0, d2*cos(theta2);  % Correct placement of cos(theta2)
       sin(theta2),  cos(theta2), 0, d2*sin(theta2);  % and sin(theta2)
                0,           0, -1, 0;               % Rotating around Z-axis
                0,           0,  0, 1];

% Transformation from frame 2 to frame 3 (T3)
H23 = [cos(theta3), 0, -sin(theta3), 0;  % Corrected sin and cos placements
       sin(theta3), 0,  cos(theta3), 0;
                0, 1,           0, 0;   % Axis aligned along Y-axis
                0, 0,           0, 1];

% Transformation from frame 3 to frame 4 (T4)
H34 = [cos(theta4), 0, -sin(theta4), 0;  % Correct rotation about Z-axis
       sin(theta4), 0,  cos(theta4), 0;
                0, -1,           0, -(d4+d3);  % Offset along Z-axis
                0,  0,           0, 1];

% Transformation from frame 4 to frame 5 (T5)
H45 = [cos(theta5), -sin(theta5), 0, d5*cos(theta5);  % Corrected cos(theta5) and sin(theta5)
       sin(theta5),  cos(theta5), 0, d5*sin(theta5);
                0,           0, -1, 0;
                0,           0,  0, 1];

% Compute transformation to each joint
H0_1 = H01;
H0_2 = H01 * H12;
H0_3 = H01 * H12 * H23;
H0_4 = H01 * H12 * H23 * H34;
H0_5 = H01 * H12 * H23 * H34 * H45;

% Prompt user to enter joint angles in degrees
theta1_deg = input('Enter theta1 (degrees): ');
theta2_deg = input('Enter theta2 (degrees): ')+270;  % Adjust theta2
theta3_deg = input('Enter theta3 (degrees): ');
theta4_deg = input('Enter theta4 (degrees): ');
theta5_deg = input('Enter theta5 (degrees): ')+90;

% Convert degrees to radians
theta1_rad = deg2rad(theta1_deg);
theta2_rad = deg2rad(theta2_deg);
theta3_rad = deg2rad(theta3_deg);
theta4_rad = deg2rad(theta4_deg);
theta5_rad = deg2rad(theta5_deg);

% Assign numerical values to link offsets
d1_val = 330;
d2_val = 475;
d3_val = 50;
d4_val = 420;
d5_val = 80;

% Create a list of symbolic variables for substitution
vars = [theta1, theta2, theta3, theta4, theta5, d1, d2, d3, d4, d5];
vals = [theta1_rad, theta2_rad, theta3_rad, theta4_rad, theta5_rad, ...
        d1_val, d2_val, d3_val, d4_val, d5_val];

% Substitute numerical values into transformations
H0_1_num = double(subs(H0_1, vars, vals));
H0_2_num = double(subs(H0_2, vars, vals));
H0_3_num = double(subs(H0_3, vars, vals));
H0_4_num = double(subs(H0_4, vars, vals));
H0_5_num = double(subs(H0_5, vars, vals));

% Extract positions of each joint
P1 = H0_1_num(1:3,4);
P2 = H0_2_num(1:3,4);
P3 = H0_3_num(1:3,4);
P4 = H0_4_num(1:3,4);
P5 = H0_5_num(1:3,4);

% Collect positions in arrays
X = [0, P1(1), P2(1), P3(1), P4(1), P5(1)];
Y = [0, P1(2), P2(2), P3(2), P4(2), P5(2)];
Z = [0, P1(3), P2(3), P3(3), P4(3), P5(3)];

% Plot the robot
figure;
plot3(X, Y, Z, 'o-', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('FANUC 200iD Robot Visualization');
axis equal;

% Add labels to each joint
for i = 1:length(X)
    text(X(i), Y(i), Z(i), sprintf('  Joint %d', i-1), 'FontSize', 8);
end

% Print the final coordinate (position of joint 5, end-effector)
fprintf('End-Effector Position (Joint 5):\n');
fprintf('Px = %.2f mm\n', P5(1));
fprintf('Py = %.2f mm\n', P5(2));
fprintf('Pz = %.2f mm\n', P5(3));
