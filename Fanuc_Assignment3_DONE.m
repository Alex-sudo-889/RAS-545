clear all;
clc;

% Define symbolic variables for joint angles and link lengths
syms t1 t2 t3 t4 t5 t6

% Link lengths (in mm)
a1 = 0;
a2 = 50;
a3 = 440;
a4 = 35;
a5 = 0;
a6 = 420;
a7 = 214;

% Define homogeneous transformations for each joint
H21 = [cos(t1),   0, -sin(t1),   a2*cos(t1);
       sin(t1),   0,  cos(t1),   a2*sin(t1);
            0,   -1,        0,         a1;
            0,    0,        0,          1];

H32 = [cos(t2),  sin(t2),    0,   a3*sin(t2);
       sin(t2), -cos(t2),    0,  -a3*cos(t2);
            0,        0,   -1,           0;
            0,        0,    0,           1];

H43 = [-sin(t3), 0, -cos(t3),     -a4*sin(t3);
       cos(t3),  0, -sin(t3),      a4*cos(t3);
            0,      -1,     0,           0;              
            0,       0,     0,           1];

H54 = [0,  cos(t4),  sin(t4),         0;
       0,  sin(t4), -cos(t4),         0;
      -1,       0,        0,        -a6;
       0,       0,        0,         1];

H65 = [cos(t5),        0, -cos(t5),  a7*cos(t5);
       sin(t5),        0, -sin(t5),  a7*sin(t5);
            0,       -1,        0,          0;              
            0,        0,        0,          1];

% Compute transformations for each joint with respect to the base
H01 = eye(4);  % Base to Joint 1
H12 = H21;
H23 = H32;
H34 = H43;
H45 = H54;
H56 = H65;

% Calculate cumulative transformations
H02 = simplify(H01 * H12);
H03 = simplify(H02 * H23);
H04 = simplify(H03 * H34);
H05 = simplify(H04 * H45);
H06 = simplify(H05 * H56);

% Prompt user for joint angles in degrees
tt1_deg = input('Enter theta1 (in degrees): ');
tt2_deg = input('Enter theta2 (in degrees): ');
tt3_input_deg = input('Enter theta3 (in degrees): ');
tt4_deg = input('Enter theta4 (in degrees): ');
tt5_deg = input('Enter theta5 (in degrees): ');
tt6_deg = input('Enter theta6 (in degrees): ');

% Convert degrees to radians
tt1 = deg2rad(tt1_deg);
tt2 = deg2rad(tt2_deg);
tt3_input = deg2rad(tt3_input_deg);

% Add offset: theta3 = theta3_input + theta2
tt3 = tt3_input + tt2;

tt4 = deg2rad(tt4_deg);
tt5 = deg2rad(tt5_deg);
tt6 = deg2rad(tt6_deg);

% Define transformation matrices with substituted angles
H02_val = vpa(subs(H02, [t1, t2, t3, t4, t5, t6], [tt1, tt2, tt3, tt4, tt5, tt6]));
H03_val = vpa(subs(H03, [t1, t2, t3, t4, t5, t6], [tt1, tt2, tt3, tt4, tt5, tt6]));
H04_val = vpa(subs(H04, [t1, t2, t3, t4, t5, t6], [tt1, tt2, tt3, tt4, tt5, tt6]));
H05_val = vpa(subs(H05, [t1, t2, t3, t4, t5, t6], [tt1, tt2, tt3, tt4, tt5, tt6]));
H06_val = vpa(subs(H06, [t1, t2, t3, t4, t5, t6], [tt1, tt2, tt3, tt4, tt5, tt6]));

% Extract XYZ positions for each joint
joint_positions = [
    0, 0, 0;                         % Base
    H02_val(1:3, 4).';               % Joint 1 position
    H03_val(1:3, 4).';               % Joint 2 position
    H04_val(1:3, 4).';               % Joint 3 position
    H05_val(1:3, 4).';               % Joint 4 position
    H06_val(1:3, 4).'];              % End Effector position

% Print calculated coordinates
fprintf('\nCalculated Joint Positions (in mm):\n');
for i = 1:size(joint_positions, 1)
    fprintf('Joint %d Position: [%.3f, %.3f, %.3f]\n', i-1, joint_positions(i, :));
end

% Plotting the robot arm in 3D based on calculated joint positions
figure;
hold on;
grid on;
axis equal;

% Plot the robot arm configuration with calculated joint positions
plot3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), 'b-o', 'LineWidth', 2, 'MarkerSize', 6);

% Customize plot appearance
legend('Robot Arm');
title('3D Robot Arm Visualization with Joint Positions');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

% Enable 3D rotation for the plot
rotate3d on;

hold off;
