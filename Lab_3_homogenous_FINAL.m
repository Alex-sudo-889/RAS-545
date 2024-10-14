clear all
clc

% Symbolic variables for joint angles (theta) and link lengths (d)
syms the1 the2 the3 the4 the5 the6 d1 d2 d3 d4 d5 d6

% Homogeneous Transformation Matrices (0H1, 1H2, etc.)
% Transformation Matrix H01

H01 = [-cos(the1),  0,             sin(the1),   0; 
       -sin(the1),  0,            -cos(the1),   0; 
        0,        -1,              0,          d1;
        0,         0,              0,           1];

% Transformation Matrix H12
H12 = [ cos(the2), -sin(the2),  0,   d2*sin(the2);
        sin(the2),  cos(the2),  0,  -d2*cos(the2);
        0,         0,           1,   0;
        0,         0,           0,   1];

% Transformation Matrix H23
H23 = [ cos(the3), -sin(the3),  0,   d3*sin(the3);
        sin(the3),  cos(the3),  0,  -d3*cos(the3);
        0,         0,           1,   0;
        0,         0,           0,   1];

% Transformation Matrix H34
H34 = [-cos(the4),  0,          sin(the4),   0; 
       -sin(the4),  0,         -cos(the4),   0; 
        0,        -1,           0,          d4;
        0,         0,           0,           1];

% Transformation Matrix H45
H45 = [ sin(the5),  0,         -cos(the5),   0;
       -cos(the5),  0,         -sin(the5),   0;
        0,        -1,           0,          d5;
        0,         0,           0,           1];

% Transformation Matrix H56
H56 = [ cos(the6), -sin(the6),  0,   0;
        sin(the6),  cos(the6),  0,   0;
        0,         0,           1,  -d6;
        0,         0,           0,   1];

% Final Transformation Matrix H06
H06 = simplify(H01 * H12 * H23 * H34 * H45 * H56);

% Extract end-effector position Px, Py, Pz
Px = H06(1, 4);
Py = H06(2, 4);
Pz = H06(3, 4);

% Enter the link lengths for MyCobot 280
d1_v = 210; d2_v = 250; d3_v = 250; d4_v = 109.5; d5_v = 107.0; d6_v = 76.2;

% Input for joint angles
disp('Enter the values for joint angles in degrees:');
theta_inputs = zeros(1,6);
for k = 1:6
    theta_inputs(k) = input(sprintf('Theta%d: ', k));
end

% Offsets in degrees
offsets = [180, 90, 0, 90, -90, 0];

% Add offsets to input angles
theta = theta_inputs + offsets;

% Convert to radians
t = deg2rad(theta);

% Assign to individual joint variables
t1 = t(1);
t2 = t(2);
t3 = t(3);
t4 = t(4);
t5 = t(5);
t6 = t(6);

% Substitute numeric values into transformation matrices
H01_num = subs(H01, [the1, d1], [t1, d1_v]);
H12_num = subs(H12, [the2, d2], [t2, d2_v]);
H23_num = subs(H23, [the3, d3], [t3, d3_v]);
H34_num = subs(H34, [the4, d4], [t4, d4_v]);
H45_num = subs(H45, [the5, d5], [t5, d5_v]);
H56_num = subs(H56, [the6, d6], [t6, d6_v]);

% Multiply the transformation matrices to get all positions
H02 = H01_num * H12_num;
H03 = H02 * H23_num;
H04 = H03 * H34_num;
H05 = H04 * H45_num;
H06 = H05 * H56_num;

% Store matrices in a cell array for visualization
H_matrices = {eye(4), H01_num, H02, H03, H04, H05, H06};

% Initialize arrays for storing joint positions
x_vals = zeros(1, 7);
y_vals = zeros(1, 7);
z_vals = zeros(1, 7);

% Extract the x, y, z coordinates for each transformation matrix
for i = 1:7
    x_vals(i) = double(H_matrices{i}(1, 4));  % Convert to double
    y_vals(i) = double(H_matrices{i}(2, 4));  % Convert to double
    z_vals(i) = double(H_matrices{i}(3, 4));  % Convert to double
end

% Add base height to the first Z position (base)
z_vals = z_vals + 0;  

% Display the final end-effector position
fprintf('The end-effector position is:\n');
fprintf('Px = %.4f mm\n', x_vals(end));
fprintf('Py = %.4f mm\n', y_vals(end));
fprintf('Pz = %.4f mm\n', z_vals(end));

% Plot the robot arm with all joint positions
figure;
plot3(x_vals, y_vals, z_vals, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
hold on;

% Plot links as lines connecting the joints
for i = 1:length(x_vals)-1
    plot3([x_vals(i) x_vals(i+1)], [y_vals(i) y_vals(i+1)], [z_vals(i) z_vals(i+1)], ...
        'LineWidth', 2, 'Color', 'r');
end

% Add base frame and end-effector markers
scatter3(x_vals(1), y_vals(1), z_vals(1), 100, 'filled', 'MarkerFaceColor', 'g');  % Base
scatter3(x_vals(end), y_vals(end), z_vals(end), 100, 'filled', 'MarkerFaceColor', 'm');  % End-effector

% Label axes and add title
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
axis equal;
title('Robot Arm Visualization');
view(3);

% Display joint numbers
for i = 1:length(x_vals)
    text(x_vals(i), y_vals(i), z_vals(i), sprintf('  Joint %d', i-1), 'FontSize', 10, 'FontWeight', 'bold');
end

hold off;
