function coords_mesh_loader()
    % Define the URDF File Path
    filename = '/Users/zandercorp/Downloads/scaling_V1.urdf';
    
    % Validate File Existence
    if ~exist(filename, 'file')
        error('URDF file does not exist: %s', filename);
    end

    % Initialize Variables
    lastModifiedTime = 0;
    showFrames = true;
    robot = [];

    % Create Figure Window
    fig = figure('Name', 'URDF Monitor and Toggle', ...
                 'NumberTitle', 'off', ...
                 'Position', [100 100 1000 800], ...
                 'CloseRequestFcn', @closeFigure, ...
                 'Color', [0.1 0.1 0.1]);

    % Create Axes for Visualization
    ax = axes('Parent', fig, 'Position', [0.1 0.15 0.85 0.8], 'Color', [0.2 0.2 0.2]);
    hold(ax, 'on');

    % Create Toggle Button
    btn = uicontrol('Parent', fig, ...
                    'Style', 'pushbutton', ...
                    'String', 'Hide Frames', ...
                    'Units', 'normalized', ...
                    'Position', [0.02 0.05 0.15 0.05], ...
                    'FontSize', 12, ...
                    'ForegroundColor', 'white', ...
                    'BackgroundColor', [0.3 0.3 0.3], ...
                    'Callback', @toggleView);

    % Initialize Camera Toolbar
    cameratoolbar(fig, 'SetMode', 'orbit');
    cameratoolbar(fig, 'Show');

    % Load and Display Initial Robot Model
    try
        robot = importrobot(filename, 'DataFormat', 'row');
        show(robot, 'Visuals', 'on', 'Frames', 'on', 'Parent', ax);
        axis(ax, 'equal');
        view(ax, 3);
        lighting gouraud
        material shiny
        camlight('headlight');
        rotate3d(ax, 'on');
        lastModifiedTime = dir(filename).datenum;
    catch ME
        disp(['Error loading URDF file: ' ME.message]);
    end

    % Create Timer for Monitoring the URDF File
    t = timer('ExecutionMode', 'fixedSpacing', ...
              'Period', 2, ...
              'TimerFcn', @checkFile);
    start(t);

    % Nested Function: Toggle Visualization Mode
    function toggleView(~, ~)
        showFrames = ~showFrames;
        if showFrames
            btn.String = 'Hide Frames';
            framesStatus = 'on';
        else
            btn.String = 'Show Frames';
            framesStatus = 'off';
        end
        cla(ax);
        try
            if ~isempty(robot)
                show(robot, 'Visuals', 'on', 'Frames', framesStatus, 'Parent', ax);
                axis(ax, 'equal');
                view(ax, 3);
                lighting gouraud
                material shiny
                camlight('headlight');
            end
        catch ME
            disp(['Error updating visualization: ' ME.message]);
        end
    end

    % Nested Function: Check for File Modifications
    function checkFile(~, ~)
        if ~isvalid(fig)
            stop(t);
            delete(t);
            return;
        end

        if exist(filename, 'file') ~= 2
            disp(['[' datestr(now) '] URDF file does not exist: ' filename]);
            return;
        end

        fileInfo = dir(filename);
        currentModTime = fileInfo.datenum;

        if currentModTime > lastModifiedTime
            try
                disp(['[' datestr(now) '] URDF file has been updated. Reloading model...']);
                robot = importrobot(filename, 'DataFormat', 'row');
                cla(ax);
                if showFrames
                    show(robot, 'Visuals', 'on', 'Frames', 'on', 'Parent', ax);
                else
                    show(robot, 'Visuals', 'on', 'Frames', 'off', 'Parent', ax);
                end
                axis(ax, 'equal');
                view(ax, 3);
                lighting gouraud
                material shiny
                camlight('headlight');
                lastModifiedTime = currentModTime;
            catch ME
                disp(['Error reloading URDF file: ' ME.message]);
            end
        end
    end

    % Nested Function: Handle Figure Closure
    function closeFigure(~, ~)
        stop(t);
        delete(t);
        delete(fig);
        disp('Figure closed. Stopped monitoring the URDF file.');
    end
end
