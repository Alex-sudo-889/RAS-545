% Clear workspace and command window
clear; clc; close all;

%% Load the Fanuc 200iD Robot Model
% Step 1: Load the robot model and set properties
urdfPath = '/Users/zandercorp/Desktop/FreeCad_stl/Fanuc_200iD_Master/URDF/lrmate200id4s.urdf';
robot = importrobot(urdfPath);
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% Step 2: Identify the End-Effector Link
% Use 'tool0' as the end-effector (since FK is calculated up to the end-effector)
endEffectorName = 'tool0';

% Step 3: Adjust Joint Limits (if necessary)
for i = 1:numel(robot.Bodies)
    if strcmp(robot.Bodies{i}.Joint.Type, 'revolute')
        robot.Bodies{i}.Joint.PositionLimits = deg2rad([-360, 360]);
    end
end



%% Set up the Inverse Kinematics Solver
% Step 4: Create the IK solver with adjusted parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 15000;
ik.SolverParameters.SolutionTolerance = 1e-6; % High precision

% Set IK weights for position and orientation
weights = [1, 1, 1, 0, 0, 0]; % Prioritize position over orientation



%% Get Desired End-Effector Position from User
% Step 5: Prompt user for desired position in mm
targetX_mm = input('Enter the desired X coordinate (in mm): ');
targetY_mm = input('Enter the desired Y coordinate (in mm): ');
targetZ_mm = input('Enter the desired Z coordinate (in mm): ');

% Convert mm to meters
targetPosition_m = [targetX_mm, targetY_mm, targetZ_mm] / 1000;

% Define the desired end-effector orientation (ZYX Euler angles in radians)
% For simplicity, we'll keep the orientation fixed at [0, 0, 0]
targetOrientation = [0, 0, 0]; % Roll, Pitch, Yaw

% Convert position and orientation to a transformation matrix
targetPose = trvec2tform(targetPosition_m) * eul2tform(targetOrientation, 'ZYX');



%% Attempt to Find a Valid Solution Within 100,000 Tries
maxAttempts = 100000;
foundSolution = false;

for attempt = 1:maxAttempts
    % Print progress every 1,000 attempts
    if mod(attempt, 1000) == 0
        fprintf('Completed %d attempts out of %d. No valid solution found yet.\n', attempt, maxAttempts);
    end

    % Use a random initial guess within joint limits
    initialGuess = randomConfiguration(robot);

    % Solve for joint angles using the IK solver
    [qSol, solInfo] = ik(endEffectorName, targetPose, weights, initialGuess);

    % Check the solution status
    if solInfo.ExitFlag <= 0
        continue; % Try the next attempt
    end

    % Convert joint angles from radians to degrees
    qSolDegrees = rad2deg(qSol);

    %% Use the FK Solver to Verify the Solution
    % Pass the joint angles to your FK solver
    % Note: Ensure that the joint order matches between the IK solver and FK solver
    % The FK solver expects joint angles in degrees

    % Adjust joint angles for FK solver conventions if necessary
    % Since in your FK code theta3 = theta3_input + theta2, we need to adjust qSolDegrees(3)
    tt1_deg = qSolDegrees(1);
    tt2_deg = qSolDegrees(2);
    tt3_input_deg = qSolDegrees(3) - tt2_deg; % Adjusted to match your FK convention
    tt4_deg = qSolDegrees(4);
    tt5_deg = qSolDegrees(5);
    tt6_deg = qSolDegrees(6);

    [computedPosition_mm] = FK_solver(tt1_deg, tt2_deg, tt3_input_deg, tt4_deg, tt5_deg, tt6_deg);

    % Compute coordinate differences
    coordinateDifferences = abs([targetX_mm, targetY_mm, targetZ_mm] - computedPosition_mm);

    % Check if differences are within 10 mm tolerance
    tolerance_mm = 10;
    if all(coordinateDifferences <= tolerance_mm)
        foundSolution = true;
        % Display results
        fprintf('Valid solution found on attempt %d!\n', attempt);

        % Display the calculated joint angles in degrees
        disp('Calculated Joint Angles (in degrees):');
        for i = 1:length(qSolDegrees)
            fprintf('Joint %d: %f degrees\n', i, qSolDegrees(i));
        end

        % Display the computed end-effector position
        disp('Computed End-Effector Position from FK (in mm):');
        fprintf('X: %.3f mm\n', computedPosition_mm(1));
        fprintf('Y: %.3f mm\n', computedPosition_mm(2));
        fprintf('Z: %.3f mm\n', computedPosition_mm(3));

        % Display coordinate differences
        disp('Coordinate Differences (Desired - Computed) in mm:');
        fprintf('ΔX: %.3f mm\n', coordinateDifferences(1));
        fprintf('ΔY: %.3f mm\n', coordinateDifferences(2));
        fprintf('ΔZ: %.3f mm\n', coordinateDifferences(3));

        disp('The computed position is within the 10 mm tolerance for all coordinates.');

        %% Visualize the Robot Configuration
        figure;
        show(robot, qSol, 'Frames', 'on', 'PreservePlot', false);
        title('Robot Configuration for Target Pose (Units in mm)');
        hold on;

        % Plot the desired target position
        plotTransforms([targetX_mm, targetY_mm, targetZ_mm], eul2quat(targetOrientation, 'ZYX'), 'FrameSize', 100);
        hold off;

        break; % Exit the loop since a valid solution is found
    else
        % If the solution is not within tolerance, do not display and continue
        continue;
    end
end

if ~foundSolution
    disp('No valid solution found within the specified attempts.');
end

%% FK Solver Function
function [end_effector_pos_mm] = FK_solver(tt1_deg, tt2_deg, tt3_input_deg, tt4_deg, tt5_deg, tt6_deg)
    % Convert degrees to radians
    tt1 = deg2rad(tt1_deg);
    tt2 = deg2rad(tt2_deg);
    tt3_input = deg2rad(tt3_input_deg);

    % Add offset: theta3 = theta3_input + theta2
    tt3 = tt3_input + tt2;

    tt4 = deg2rad(tt4_deg);
    tt5 = deg2rad(tt5_deg);
    tt6 = deg2rad(tt6_deg);

    % Link lengths (in mm)
    a1 = 0;
    a2 = 50;
    a3 = 440;
    a4 = 35;
    a5 = 0;
    a6 = 420;
    a7 = 80;

    % Define homogeneous transformations for each joint
    H21 = [cos(tt1),   0, -sin(tt1),   a2*cos(tt1);
           sin(tt1),   0,  cos(tt1),   a2*sin(tt1);
                0,   -1,        0,         a1;
                0,    0,        0,          1];

    H32 = [cos(tt2),  sin(tt2),    0,   a3*sin(tt2);
           sin(tt2), -cos(tt2),    0,  -a3*cos(tt2);
                0,        0,   -1,           0;
                0,        0,    0,           1];

    H43 = [-sin(tt3), 0, -cos(tt3),     -a4*sin(tt3);
           cos(tt3),  0, -sin(tt3),      a4*cos(tt3);
                0,      -1,     0,           0;              
                0,       0,     0,           1];

    H54 = [0,  cos(tt4),  sin(tt4),         0;
           0,  sin(tt4), -cos(tt4),         0;
          -1,       0,        0,        -a6;
           0,       0,        0,         1];

    H65 = [cos(tt5),        0, -cos(tt5),  a7*cos(tt5);
           sin(tt5),        0, -sin(tt5),  a7*sin(tt5);
                0,       -1,        0,          0;              
                0,        0,        0,          1];

    % Compute cumulative transformations
    H01 = eye(4);  % Base to Joint 1
    H12 = H21;
    H23 = H32;
    H34 = H43;
    H45 = H54;
    H56 = H65;

    H02 = H01 * H12;
    H03 = H02 * H23;
    H04 = H03 * H34;
    H05 = H04 * H45;
    H06 = H05 * H56;

    % Extract end-effector position
    end_effector_pos_mm = H06(1:3, 4).';
end
