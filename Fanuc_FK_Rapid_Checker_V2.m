% FANUC 200iD Forward Kinematics and Full Visualization with Preloaded Data

clear all; % Clear and close figures at the start of the program

% Preload joint angles and XYZ from real robot data
% Ignoring J6 since it doesn't affect the XYZ of the end-effector
joint_data = [
    21.014, -14.366, 0.179, 55.049, 19.516, 503.880, 256.335, 504.149;
    4.469, -32.800, 29.847, 19.476, 15.436, 308.287, 43.141, 758.473;
    -53.636, -33.209, 25.420, 25.876, -18.375, 203.269, -325.721, 612.394
];

% Define symbolic variables for joint angles and link offsets
syms theta1 theta2 theta3 theta4 theta5
syms d1 d2 d3 d4 d5

% Specify theta offsets (in degrees) for each joint
theta_offsets = [0, 270, 0, 0, -90];  % Adjust theta2 by 180 degrees, for example

% Homogeneous Transformation Matrices between each link
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
                0, -1,           0, (d4+d3);  % Offset along Z-axis
                0,  0,           0, 1];

% Transformation from frame 4 to frame 5 (T5)
H45 = [cos(theta5), -sin(theta5), 0, d5*cos(theta5);  % Corrected cos(theta5) and sin(theta5)
       sin(theta5),  cos(theta5), 0, d5*sin(theta5);
                0,           0, -1, 0;
                0,           0,  0, 1];

% Assign numerical values to link offsets
d1_val = 330;
d2_val = 475;
d3_val = 50;
d4_val = 420;
d5_val = 80;

% Create a list of symbolic variables for substitution
vars = [theta1, theta2, theta3, theta4, theta5, d1, d2, d3, d4, d5];

% Iterate over each joint configuration from real robot data
for i = 1:size(joint_data, 1)
    % Extract joint angles and real XYZ coordinates
    theta1_deg = joint_data(i, 1) + theta_offsets(1);
    theta2_deg = joint_data(i, 2) + theta_offsets(2);  % Applying theta2 offset
    theta3_deg = joint_data(i, 3) + theta_offsets(3);
    theta4_deg = joint_data(i, 4) + theta_offsets(4);
    theta5_deg = joint_data(i, 5) + theta_offsets(5);
    real_X = joint_data(i, 6);
    real_Y = joint_data(i, 7);
    real_Z = joint_data(i, 8);
    
    % Convert degrees to radians
    theta1_rad = deg2rad(theta1_deg);
    theta2_rad = deg2rad(theta2_deg);
    theta3_rad = deg2rad(theta3_deg);
    theta4_rad = deg2rad(theta4_deg);
    theta5_rad = deg2rad(theta5_deg);
    
    % Substitute numerical values into transformations
    vals = [theta1_rad, theta2_rad, theta3_rad, theta4_rad, theta5_rad, ...
            d1_val, d2_val, d3_val, d4_val, d5_val];

    % Compute transformations for each joint
    H0_1_num = double(subs(H01, vars, vals));
    H0_2_num = double(subs(H01 * H12, vars, vals));
    H0_3_num = double(subs(H01 * H12 * H23, vars, vals));
    H0_4_num = double(subs(H01 * H12 * H23 * H34, vars, vals));
    H0_5_num = double(subs(H01 * H12 * H23 * H34 * H45, vars, vals));
    
    % Extract the calculated end-effector position
    calc_X = H0_5_num(1,4);
    calc_Y = H0_5_num(2,4);
    calc_Z = H0_5_num(3,4);
    
    % Calculate differences
    diff_X = real_X - calc_X;
    diff_Y = real_Y - calc_Y;
    diff_Z = real_Z - calc_Z;
    
    % Print the results
    fprintf('\nConfiguration %c:\n', 'A' + i - 1);
    fprintf('Given Coordinates:    [%.3f, %.3f, %.3f]\n', real_X, real_Y, real_Z);
    fprintf('Calculated Coordinates: [%.3f, %.3f, %.3f]\n', calc_X, calc_Y, calc_Z);
    fprintf('Difference:            [%.3f, %.3f, %.3f]\n', diff_X, diff_Y, diff_Z);

    % Collect positions of all joints for plotting
    P1 = H0_1_num(1:3,4);
    P2 = H0_2_num(1:3,4);
    P3 = H0_3_num(1:3,4);
    P4 = H0_4_num(1:3,4);
    P5 = H0_5_num(1:3,4);

    % Collect positions in arrays for each configuration
    X = [0, P1(1), P2(1), P3(1), P4(1), P5(1)];
    Y = [0, P1(2), P2(2), P3(2), P4(2), P5(2)];
    Z = [0, P1(3), P2(3), P3(3), P4(3), P5(3)];
    
    % Plot the entire robot arm with joints in a new figure for each configuration
    figure(i);  % Create a new figure for each configuration
    plot3(X, Y, Z, 'o-', 'LineWidth', 2, 'MarkerSize', 6);
    grid on;
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title(sprintf('FANUC 200iD Configuration %c', 'A' + i - 1));
    axis equal;
    
    % Add labels to joints for this configuration
    for j = 1:length(X)
        text(X(j), Y(j), Z(j), sprintf('  Joint %d (C%c)', j-1, 'A' + i - 1), 'FontSize', 8);
    end
end
