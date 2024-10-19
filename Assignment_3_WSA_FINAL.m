% MATLAB Code for Workspace Analysis with User Options

% Step 1: Import the URDF File into MATLAB
meshPath = '/Users/zandercorp/Desktop/FreeCad_stl/Fanuc_200iD_Master'; % Adjust if necessary
robot = importrobot('/Users/zandercorp/Desktop/FreeCad_stl/Fanuc_200iD_Master/URDF/lrmate200iD4s.urdf', 'MeshPath', meshPath);
robot.DataFormat = 'row'; % Use row format for compatibility
robot.Gravity = [0 0 -9.81]; % Set gravity if dynamics are considered

% Verify that the robot and meshes are loaded correctly
figure;
show(robot);
title('Imported Fanuc 200iD Robot with Meshes');

% Step 2: Generate the Workspace
% Use generateRobotWorkspace to get workspace points and configurations
[wksp, cfgs] = generateRobotWorkspace(robot, {});

% Step 3: Present the Menu to the User
disp('Please choose an option for workspace analysis:');
disp('1. Yoshikawa index');
disp('2. Inverse condition index');
disp('3. Asada index');
disp('4. Workspace analysis using all motion components');
disp('5. Workspace analysis with x- and y- linear components');
disp('6. Non-voxelized workspace analysis');
disp('7. Run all analyses');

choice = input('Enter your choice (1-7): ');

% Step 4: Perform the Selected Analysis

switch choice
    case 1 % Yoshikawa index
        indexType = 'yoshikawa';
        motionComponent = 'combined';
        voxelize = true;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'Yoshikawa Index';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 2 % Inverse condition index
        indexType = 'inverse-condition';
        motionComponent = 'combined';
        voxelize = true;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'Inverse Condition Index';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 3 % Asada index
        indexType = 'asada';
        motionComponent = 'linear'; % As per MATLAB example
        voxelize = true;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'Asada Index';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 4 % Workspace analysis using all motion components
        indexType = 'yoshikawa'; % Default index type
        motionComponent = 'combined';
        voxelize = true;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'All Motion Components';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 5 % Workspace analysis with x- and y- linear components
        indexType = 'yoshikawa'; % Default index type
        motionComponent = [0 0 0 1 1 0]; % x and y linear components
        voxelize = true;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'Workspace Analysis with X- and Y-Linear Components';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 6 % Non-voxelized workspace analysis
        indexType = 'yoshikawa'; % Default index type
        motionComponent = 'combined';
        voxelize = false;
        mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', indexType, 'MotionComponent', motionComponent);
        mIndex = max(mIndex, 0); % Ensure non-negative values
        titleText = 'Non-Voxelized Workspace Analysis';
        visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize);

    case 7 % Run all analyses
        analyses = [
            struct('indexType', 'yoshikawa', 'motionComponent', 'combined', 'voxelize', true, 'titleText', 'Workspace Analysis using Yoshikawa Index');
            struct('indexType', 'inverse-condition', 'motionComponent', 'combined', 'voxelize', true, 'titleText', 'Workspace Analysis using Inverse Condition Index');
            struct('indexType', 'asada', 'motionComponent', 'linear', 'voxelize', true, 'titleText', 'Workspace Analysis using Asada Index');
            struct('indexType', 'yoshikawa', 'motionComponent', 'combined', 'voxelize', true, 'titleText', 'Workspace Analysis using All Motion Components');
            struct('indexType', 'yoshikawa', 'motionComponent', [0 0 0 1 1 0], 'voxelize', true, 'titleText', 'Workspace Analysis with X- and Y-Linear Components');
            struct('indexType', 'yoshikawa', 'motionComponent', 'combined', 'voxelize', false, 'titleText', 'Non-Voxelized Workspace Analysis');
        ];
        for a = analyses'
            mIndex = manipulabilityIndex(robot, cfgs, 'IndexType', a.indexType, 'MotionComponent', a.motionComponent);
            mIndex = max(mIndex, 0); % Ensure non-negative values
            visualizeWorkspace(robot, cfgs, wksp, mIndex, a.titleText, a.voxelize);
        end

    otherwise
        disp('Invalid choice. Please run the script again and select a valid option.');
end

% Function to visualize workspace
function visualizeWorkspace(robot, cfgs, wksp, mIndex, titleText, voxelize)
    figure;
    show(robot, cfgs(1,:));
    hold on;
    % Invert the colormap so that blue is high values, red is low values
    cmap = jet(256);
    cmap = flipud(cmap); % Invert the colormap
    colormap(cmap);
    % Visualize workspace with inverted colormap
    showWorkspaceAnalysis(wksp, mIndex, 'Voxelize', voxelize);
    hold off;
    title(titleText);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    grid on;
    view(3);
    colorbar; % Display colorbar for manipulability index values
    % Label the figure window
    set(gcf, 'Name', titleText, 'NumberTitle', 'off');
end
