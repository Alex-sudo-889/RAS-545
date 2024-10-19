% MATLAB Code for Inverse Kinematics with Fanuc Robot (Simplified)

% Step 1: Import the URDF File
meshPath = '/Users/zandercorp/Desktop/FreeCad_stl/Fanuc_200iD_Master'; % Adjust if necessary
robot = importrobot('/Users/zandercorp/Desktop/FreeCad_stl/Fanuc_200iD_Master/URDF/lrmate200iD4s.urdf', 'MeshPath', meshPath);
robot.DataFormat = 'row';  % Use 'row' data format
robot.Gravity = [0 0 -9.81];

% Verify that the robot and meshes are loaded correctly
figure;
show(robot);
title('Imported Fanuc 200iD Robot with Meshes');

% Step 2: Define Given Angles and Positions from the Table

% Joint angles (degrees)
givenAngles_deg = [
    21.014,  -14.366,   0.179,  55.049,  19.516, 259.652;
     4.469,  -32.800,  29.847,  19.476,  15.436, 116.611;
   -53.636,  -33.209,  25.420,  25.876, -18.375, 116.611
];

% Desired XYZ positions (mm to meters)
desiredPositions = [
    503.880,  256.335,  504.149;
    308.287,   43.141,  758.473;
    203.269, -325.721,  612.394
] / 1000;  % Convert mm to meters

% Desired orientations (Yaw, Pitch, Roll in degrees)
desiredOrientations_deg = [
    -73.991, -45.195, -64.283;
     33.224,  33.524,  61.443;
     76.585,  49.329,  18.110
];

% Step 3: Function to solve IK and compute angles, positions, and differences
function computeIKFromTable(setNum, robot, desiredPosition, givenAngles_deg, desiredOrientation_deg)
    % Define the Desired End-Effector Pose
    desiredOrientation_rad = deg2rad(desiredOrientation_deg);  % Convert to radians
    desiredPose = trvec2tform(desiredPosition) * eul2tform(desiredOrientation_rad, 'ZYX');  % Use 'ZYX' convention

    % Using the inverseKinematics Solver
    ik = inverseKinematics('RigidBodyTree', robot);
    endEffector = robot.BodyNames{end};  % End-effector link
    weights = [1, 1, 1, 1, 1, 1];  % Weights for position and orientation
    initialGuess = deg2rad(givenAngles_deg);  % Convert given angles to radians

    % Solve for the joint configuration
    [configSol, solInfo] = ik(endEffector, desiredPose, weights, initialGuess);

    % Extract computed joint positions
    computedAngles_deg = rad2deg(configSol);  % Convert to degrees

    % Calculate Difference in Angles
    angleDifference_deg = givenAngles_deg - computedAngles_deg;

    % Print Results
    fprintf('Set %d:\n', setNum);
    fprintf('Given Angles    : [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', givenAngles_deg);
    fprintf('Computed Angles : [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', computedAngles_deg);
    fprintf('Angle Difference: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n\n', angleDifference_deg);

    % Get the end-effector position from the inverse kinematics solution
    endEffectorPoseIK = getTransform(robot, configSol, endEffector);
    endEffectorPositionIK = tform2trvec(endEffectorPoseIK);

    % Calculate Difference in Position (in millimeters)
    positionDifference_mm = (endEffectorPositionIK - desiredPosition) * 1000;  % Convert to mm

    % Print XYZ Positions
    fprintf('Given XYZ    : [%.3f, %.3f, %.3f] mm\n', desiredPosition * 1000);
    fprintf('Computed XYZ : [%.3f, %.3f, %.3f] mm\n', endEffectorPositionIK * 1000);
    fprintf('Position Difference: [%.3f, %.3f, %.3f] mm\n\n', positionDifference_mm);
end

% Step 4: Iterate through all sets and solve IK
for i = 1:size(givenAngles_deg, 1)
    % Solve IK for each set of given angles and positions
    computeIKFromTable(i, robot, desiredPositions(i, :), givenAngles_deg(i, :), desiredOrientations_deg(i, :));
end
