function [actuatorCommands, data] = Autonomy(state, LidarScan, cameraImage, dock, data)
% Autonomous USV actuator control using a Finite State Machine (FSM) approach.
% Authors: Thomas Jones & Charlotte Downs
% Course: ME 503: Unmanned Vehicle Systems, Fall 2023
% Instructor: Dr. Eric Coyle
% Submitted 12/12/2023

% Configuration for enabling/disabling objectives
% These flags control which tasks are active in the FSM.
debugMode = true; % Activate debug mode
disp(dock);
config.enableNavigationChannel = true; % Fully implemented, well tested
config.enableObstacleChannel = true; % Fully implemented, well tested
config.enableObstacleField = false; % Not to be attempted
config.enableDocking = true; % Finished, needs implementation and testing
config.enableSpeedGate = false; % Not to be attempted
config.enableReturnToHome = true; % Fully implemented, needs testing

config.enableDebug = false;

% Initialization
% Check if data is empty and initialize if necessary.
if isempty(data)
    data = initializeData(state, debugMode);
end
actuatorCommands = [0 0];
if config.enableDebug
    data.state.current = 'Configuration mode';
end

% disp(state.time) % Output sim time

% FSM for task management, decisions, and waypoint generation
% The FSM controls the ASV's behavior based on the current state.
switch data.state.current
    case 'NavigationChannel'
        % Handle navigation channel task
        if config.enableNavigationChannel
            data = handleNavigationChannel(data, state, cameraImage);
        else
            data.state.current = 'ObstacleChannel';
            data.state.previous = 'Start';
            data.firstRun = true;
        end
    case 'ObstacleChannel'
        % Handle obstacle channel task
        if config.enableObstacleChannel
            data = handleObstacleChannel(data, state, cameraImage);
        else
            data.state.current = 'ObstacleField';
            data.state.previous = 'NavigationChannel';
        end
    case 'ObstacleField'
        % Handle obstacle field task
        if config.enableObstacleField
            data = handleObstacleField(data, state);
        else
            data.state.current = 'Docking';
            data.state.previous = 'ObstacleField';
        end
    case 'Docking'
        % Handle docking task

        if config.enableDocking
            [actuatorCommands, data] = handleDocking(data, state, cameraImage, dock);
        else
            data.state.current = 'SpeedGate';
            data.state.previous = 'Docking';
            data.dockingCommandsOverride = true;
            data.collisionAvoidance.on = true;
        end
    case 'SpeedGate'
        % Handle speed gate task
        if config.enableSpeedGate
            data = handleSpeedGate(data, state);
        else
            data.state.current = 'ReturnToHome';
            data.dockingCommandsOverride = false;
            data.state.previous = 'SpeedGate';
        end
    case 'ReturnToHome'
        % Handle return to home task
        data.dockingCommandsOverride = false;
        data.navigation.waypoint = data.returnHome.startLocation;
        data.GPS.CEP = 0.12;
    case 'Complete'
        % Indicate mission completion
        disp('Mission complete');
    otherwise
        % Handle debug state
        warning('debug objective');
end

% Follow next commanded waypoint with collision avoidance
% followWaypoint(data, state, LiDAR_Points, steering gain, fthrust)
if ~data.dockingCommandsOverride
    [actuatorCommands, data] = followWaypoint(data, state, LidarScan,...
        data.actuator.steeringGain, data.actuator.fthrust);
end
% Supporting functions below (initializeAutonomy, processNavigationChannel, etc.)
end

%% Initialize
function data = initializeData(state, debugMode)
% Initialize state variables and data structures for the Autonomy.m function.
fprintf('Initializing data...\n');

% Initialize the data structure
data = struct();

% Flag for the first run
data.firstRun = true;

% Debugging and Logging
if debugMode
    data.debug.mode = true; % Debug mode flag
    data.debug.plotted_waypoints = 0;
    data.debug.circle_handles = gobjects(0);
end

% State Flags
data.state = struct('current', 'NavigationChannel',...
    'previous', '',...
    'safeMode', false, ...
    'searchMode', false);

% Sensor Info
data.GPS = struct('CEP', 0.25); % Circular Error Probable from GPS reading
data.lidarPoints = [];
data.timeStamps = [];

% Waypoint and Navigation Data
data.navigation = struct(...
    'waypoint', [], ...                      % Waypoints array
    'startingWaypoint', [state.Pos(1);state.Pos(2)], ...
    'waypointIdx', 1, ...             % Index of current waypoint
    'eTheta', [0 0], ...                     % Error in theta (yaw error)
    'currentPoint', struct('XData', [], 'YData', []), ... % Struct for current point data
    'CEP', struct('XData', [], 'YData', []));         % Struct for CEP data

% Actuator Status and Control
data.actuator = struct(...
    'fthrust', 0.8,...
    'steeringGain', 0.6);
data.eTheta = [0; 0];

% Collision Avoidance
DetectionGrid_sensor = [0,0;0.1,0.6;1.3,0.6;1.3,-0.6;0.1,-0.6;0,0];
data.collisionAvoidance = struct(...
    'on', true, ...
    'activelyAvoiding', false, ...
    'safeDistance', 0.35, ...
    'turnDirection', '', ...
    'DetectionGrid_FRD', DetectionGrid_sensor + [0.27, 0]);

% Nav Channel
data.navChannel.detector = [];
data.navChannelDistLimit = 20;
data.navChannelTimeLimit = 20;

% Obstacle Channel
data.obstChannelDistLimit = 70;
data.obstChannelTimeLimit = 100;

% Load obstacle detection model
clear detector;
load('BoatYoloNetworkSmallBouy','detector','info');
data.obstChannel = struct();
data.obstChannel.detector=detector;
data.obstChannel.info=info;
data.navigation.waypoint = [10 30];
data.actuator.fthrust = 0.3;
data.actuator.steeringGain = 0.9;
data.firstRun = false;

% Docking
data.waypointReached = false;
data.stopDockNav = 0;
data.dockingCommandsOverride = false;

% Return to Home
data.returnHome = struct(...
    'currentState', 'NAVIGATING_HOME', ...
    'distanceTraveled', 0, ...
    'startLocation', [state.Pos(1), state.Pos(2)], ...
    'completionThreshold', 2);
fprintf('Data initialized successfully.\n');
end


%% Navigation Channel
function data = handleNavigationChannel(data, state, cameraImage)
if isempty(data.navChannel.detector)
    fprintf('Mission starting...\n');
    load('NavChannelYOLO','detector','info');
    data.navChannel.detector=detector;
    data.navChannel.info=info;
    data.navigation.waypoint = [10 10];
    data.firstRun = false;
end

%Run the Deep Learning detector
[bboxes,scores, label] = detect(data.navChannel.detector,cameraImage,'Threshold',0.5);

%Place annotations on image and get class list
categories=categorical({'Tall_Green','Tall_Red','Tall_White'});
colorSpec=[0 1 0;
    1 0 0;
    1 1 1;
    1 1 0;
    0 0 1];
label_num=zeros(length(scores));
for k=1:length(scores)

    minIdx = find(label(k)==categories);
    label_num(k) = minIdx;

    annotation = sprintf('%s %4.2f',label(k), scores(k));
    cameraImage = insertObjectAnnotation(cameraImage,'rectangle',bboxes(k,:),...
        annotation,'FontSize',10,'Color',uint8(255*colorSpec(minIdx,:)));
end

% Begin waypoint plotting logic from camera data
% label_num 2 = Green, 1 = Red

% Find indices of green and red buoys
greenIdx = find(label_num == 2);
redIdx = find(label_num == 1);

% Check if exactly one green and one red buoy are detected
if length(greenIdx) == 1 && length(redIdx) == 1

    % Define camera and image properties
    cameraPosition_FRD = [0.27, 0, 0]; % Camera position in FRD frame (m)
    pixelPitch = 0.0138e-3; % Pixel pitch (m/pixel)
    buoyHeightReal = 2.2; % Actual height of the buoy (m) OFFSET!
    focalLength = 2.46e-3; % Focal length of the camera (m)
    imageSize = [618, 464]; % Image size in pixels [width, height]

    % Initialize array to store buoy coordinates in NED frame
    distances_NED = zeros(2, 3); % [X, Y, Z] for green and red

    % Combine indices for iterating over green and red buoys
    buoyIdx = [greenIdx, redIdx];

    for i = 1:length(buoyIdx)
        idx = buoyIdx(i);
        bbox = bboxes(idx, :);

        % Calculate pixel coordinates of buoy center
        buoyCenterX = bbox(1) + bbox(3)/2;
        buoyCenterY = bbox(2) + bbox(4)/2;

        % Calculate distances from buoy center to top and bottom of bounding box in sensor coordinates
        sensorTopY = (bbox(2) - buoyCenterY) * pixelPitch;
        sensorBottomY = (bbox(2) + bbox(4) - buoyCenterY) * pixelPitch;

        % Calculate distance to buoy using pinhole camera model
        d = (buoyHeightReal * focalLength) / (sensorBottomY - sensorTopY);

        % Calculate left/right distance from image center to buoy center in pixels
        deltaX_pixels = buoyCenterX - imageSize(1)/2;

        % Convert pixel distance to meters on image sensor
        deltaX_meters = deltaX_pixels * pixelPitch;

        % Calculate left/right position of buoy in FRD frame
        distances_FRD = [d, (deltaX_meters * d) / focalLength, 0] + cameraPosition_FRD;

        % Construct rotation matrix from FRD to NED frame
        yaw = state.rot(3);
        R_FRD_to_NED = [cos(yaw), sin(yaw), 0; -sin(yaw), cos(yaw), 0; 0, 0, 1];

        % Transform buoy position to NED frame
        distances_NED(i, :) = R_FRD_to_NED\distances_FRD';
    end

    % Calculate average waypoint location in NED frame
    waypoint_NED = state.Pos + mean(distances_NED);
    if ~isempty(data.navigation.waypoint)
        if norm(abs(waypoint_NED(1:2)' - data.navigation.waypoint')) > 1
            % Store waypoint in data structure
            data.navigation.waypoint = waypoint_NED(1:2);
        end
    else
        data.navigation.waypoint = waypoint_NED(1:2);
    end
else
    % fprintf('No buoy pair detected.\n');
end

data.firstRun = false;
if state.Pos(2) > data.navChannelDistLimit || state.time > data.navChannelTimeLimit
    data.state.current = 'ObstacleChannel';
end
end

%% Obstacle Channel
function data = handleObstacleChannel(data, state, cameraImage)
%Run the Deep Learning detector
[bboxes,scores, label] = detect(data.obstChannel.detector,cameraImage,'Threshold',0.5);
data.actuator.fthrust = 0.6;
%Place annotations on image and get class list
categories=categorical({'SmallGreen','SmallRed','SmallYellow'});
colorSpec=[0 1 0;
    1 0 0;
    1 1 1;
    1 1 0;
    0 0 1];
label_num=zeros(length(scores));
for k=1:length(scores)

    minIdx = find(label(k)==categories);
    label_num(k) = minIdx;

    annotation = sprintf('%s %4.2f',label(k), scores(k));
    cameraImage = insertObjectAnnotation(cameraImage,'rectangle',bboxes(k,:),...
        annotation,'FontSize',10,'Color',uint8(255*colorSpec(minIdx,:)));
end

% Begin waypoint plotting logic from camera data
% label_num 2 = Green, 1 = Red

% Find indices of green and red buoys
greenIdx = find(label_num == 2);
redIdx = find(label_num == 1);

% Check if exactly one green and one red buoy are detected
if length(greenIdx) == 1 && length(redIdx) == 1

    % Define camera and image properties
    cameraPosition_FRD = [0.27, 0, 0]; % Camera position in FRD frame (m)
    pixelPitch = 0.0138e-3; % Pixel pitch (m/pixel)
    buoyHeightReal = 0.4; % Actual height of the buoy (m) OFFSET!
    focalLength = 2.46e-3; % Focal length of the camera (m)
    imageSize = [618, 464]; % Image size in pixels [width, height]

    % Initialize array to store buoy coordinates in NED frame
    distances_NED = zeros(2, 3); % [X, Y, Z] for green and red

    % Combine indices for iterating over green and red buoys
    buoyIdx = [greenIdx, redIdx];

    for i = 1:length(buoyIdx)
        idx = buoyIdx(i);
        bbox = bboxes(idx, :);

        % Calculate pixel coordinates of buoy center
        buoyCenterX = bbox(1) + bbox(3)/2;
        buoyCenterY = bbox(2) + bbox(4)/2;

        % Calculate distances from buoy center to top and bottom of bounding box in sensor coordinates
        sensorTopY = (bbox(2) - buoyCenterY) * pixelPitch;
        sensorBottomY = (bbox(2) + bbox(4) - buoyCenterY) * pixelPitch;

        % Calculate distance to buoy using pinhole camera model
        d = (buoyHeightReal * focalLength) / (sensorBottomY - sensorTopY);

        % Calculate left/right distance from image center to buoy center in pixels
        deltaX_pixels = buoyCenterX - imageSize(1)/2;

        % Convert pixel distance to meters on image sensor
        deltaX_meters = deltaX_pixels * pixelPitch;

        % Calculate left/right position of buoy in FRD frame
        distances_FRD = [d, (deltaX_meters * d) / focalLength, 0] + cameraPosition_FRD;

        % Construct rotation matrix from FRD to NED frame
        yaw = state.rot(3);
        R_FRD_to_NED = [cos(yaw), sin(yaw), 0; -sin(yaw), cos(yaw), 0; 0, 0, 1];

        % Transform buoy position to NED frame
        distances_NED(i, :) = R_FRD_to_NED\distances_FRD';
    end

    % Calculate average waypoint location in NED frame
    waypoint_NED = state.Pos + mean(distances_NED);

    % Check if the new waypoint is valid based on its second value
    if ~isempty(data.navigation.waypoint)
        % Compare the second value of the new and existing waypoints
        if waypoint_NED(2) >= data.navigation.waypoint(2)
            % Update waypoint only if the new waypoint's second value is greater or equal
            if norm(abs(waypoint_NED(1:2)' - data.navigation.waypoint')) > 1
                data.navigation.waypoint = waypoint_NED(1:2);
            end
        end
    else
        % If there is no existing waypoint, set the new waypoint
        data.navigation.waypoint = waypoint_NED(1:2);
    end
else
    % fprintf('No buoy pair detected.\n');
end

data.firstRun = false;
if state.Pos(2) > data.obstChannelDistLimit || state.time > data.obstChannelTimeLimit
    data.state.current = 'Docking';
end
end

%% Docking
function [actuatorCommands, data] = handleDocking(data, state, cameraImage, dock)
if state.time > 200 || state.Pos(1) > 37 || state.Pos(2) > 55 || state.Pos(2) < 30 || state.Pos(1) < 17
    fprintf('Ran out of time. Returning home...\n');
    data.state.current = 'ReturnToHome';
    actuatorCommands = [0 0];
end
actuatorCommands = [0 0];
data.navigation.waypoint = [20, 50];
if ~data.waypointReached
    data.collisionAvoidance.on = true;
    distance = norm(data.navigation.waypoint - state.Pos(1:2));

    if distance < (data.GPS.CEP + 0.1)
        data.waypointReached = true;
    end
else
    data.dockingCommandsOverride = true;
    data.collisionAvoidance.on = false;
    actuatorCommands = [0.1, -0.1];

    limits = [0.47 0.51];
    [signMaskImage, masks] = findSigns(cameraImage);
    labelFigure(signMaskImage, cameraImage, masks, limits);

    %% Main Dock logic
    % uses gain controller solution from waypoint assignment
    if ~isempty(dock)

        switch dock
            case 'R' % red dock activated
                stats = regionprops(masks.red, 'Centroid','BoundingBox','Area');
                offset = -1;
                heightThreshold = 85;
                heightThreshold2 = 30;
                areaThreshold = 2000; %6000;

            case 'G' % green dock activated
                stats = regionprops(masks.green, 'Centroid','BoundingBox','Area');
                offset = -0.65; %-0.15;
                heightThreshold = 80;
                heightThreshold2 = 45;
                areaThreshold = 2000; %8000;

            case 'B' % blue dock activated
                stats = regionprops(masks.blue, 'Centroid','BoundingBox','Area');
                offset = -1.1;
                heightThreshold = 90;
                heightThreshold2 = 30;
                areaThreshold = 2000; %6000;
            otherwise
                stats = [];
        end

        if ~isempty(stats)

            if data.stopDockNav == 0 % navigating to dock

                [~, idx] = max([stats.Area]);
                targetPos = stats(idx).Centroid;
                BBox = stats(idx).BoundingBox;

                dN = targetPos(2) - state.Pos(1);
                dE = targetPos(1) - state.Pos(2);

                area = stats(idx).Area;
                height = BBox(4);

                % updates the offset for the green dock to keep it from turning
                % when its close to the dock
                if dock == 'G' && area > 2000
                    offset = -1;
                end

                yawDesired = atan2(dE, dN) + offset;
                yawError = yawDesired - state.rot(3);

                if height < heightThreshold %area < areaThreshold

                    if data.stopDockNav == 0

                        while yawError > pi
                            yawError = yawError - 2 * pi;
                        end
                        while yawError < -pi
                            yawError = yawError + 2 * pi;
                        end

                        KpT = 0.1;
                        data.utheta = KpT * yawError;

                        if data.actuator.fthrust + data.utheta > 1
                            fthrust = 1 - data.utheta;
                        else
                            fthrust = data.actuator.fthrust;
                        end

                        actuatorCommands = [fthrust + data.utheta, fthrust - data.utheta];

                        data.eTheta = [yawError, data.eTheta(1)];
                    else
                        actuatorCommands = [-1 -1];
                        data.stopDockNav = 1; % flag that boat has docked
                    end
                else
                    actuatorCommands = [-1 -1];
                    data.stopDockNav = 1; % flag boat has docked
                end

            end

            if data.stopDockNav == 1 % after boat has docked
                % reversing from dock
                [~, idx] = max([stats.Area]);
                %area = stats(idx).Area;
                BBox = stats(idx).BoundingBox;
                height = BBox(4)
                if height > heightThreshold2 %area > 500
                    actuatorCommands = [-1 -1];
                else
                    actuatorCommands = [0 0]; % return home
                end

            end
        end

    end
end
    function [signMaskImage, masks] = findSigns(cameraImage)
        % findSigns - Function to identify and mask dock signs based on their color.

        hsvImage = rgb2hsv(cameraImage); % Convert to HSV format

        % Define color thresholds (from color thresholder app)
        colorDefs = struct(...
            'green', struct('h', [0.310 0.351], 's', [0.543 1.000], 'v', [0.142 1.000]), ...
            'red1', struct('h', [0 0.05], 's', [0.2 1.000], 'v', [0.2 1.000]), ...
            'red2', struct('h', [0.95 1], 's', [0.2 1.000], 'v', [0.2 1.000]), ...
            'blue', struct('h', [0.654 0.681], 's', [0.317 1.000], 'v', [0.207 1.000]) ...
            ); % Split up the red into two seperate ends of the HSV spectrum

        % Initialize mask and process each color
        masks = struct();
        mask = false(size(hsvImage, 1), size(hsvImage, 2));
        for colorName = fieldnames(colorDefs)'
            colorMask = getColorMask(hsvImage, colorDefs.(colorName{1}));
            if strcmp(colorName, 'red1') || strcmp(colorName, 'red2') % Handle red
                if isfield(masks, 'red')
                    masks.red = masks.red | colorMask;
                else
                    masks.red = colorMask;
                end
            else
                masks.(colorName{1}) = colorMask;
            end
            mask = mask | colorMask;
        end
        signMaskImage = mask;   % Final mask produced from colors
    end

    function colorMask = getColorMask(hsvImage, colorDef)
        % getColorMask - Function to get the color mask of a given color definition.

        colorMask = hsvImage(:,:,1) >= colorDef.h(1) & hsvImage(:,:,1) <= colorDef.h(2) ...
            & hsvImage(:,:,2) >= colorDef.s(1) & hsvImage(:,:,2) <= colorDef.s(2) ...
            & hsvImage(:,:,3) >= colorDef.v(1) & hsvImage(:,:,3) <= colorDef.v(2);
    end

    function labelFigure(rectanglesMaskImage, cameraImage, masks, limits)
        % labelFigure - Function to label the detected rectangles on the original image.

        % Extract contours
        [B,~] = bwboundaries(rectanglesMaskImage, 'noholes');

        % Prepare figure
        f = figure(3);  % Plot on top of the existing camera image
        if isempty(f.Children)  % Find and remove existing boxes, labels if needed
            imshow(cameraImage);
            hold on;
        else
            delete(findobj(f, 'Type', 'Rectangle'));
            delete(findobj(f, 'Type', 'Text'));
        end

        % Define Y-coordinate image constraints to limit image detection to height
        % this ensures that the tree leaves aren't detected.
        upperYLimit = size(cameraImage, 1) * limits(1);
        lowerYLimit = size(cameraImage, 1) * limits(2);

        % Identify and label only largest area for each color, helping with false
        % buoy detection. This ensures only the circular signs are detected.
        regionProps = regionprops(rectanglesMaskImage, 'Centroid', 'BoundingBox', 'Area');
        largestAreasForColors = getLargestAreasForColors(masks, B, regionProps, upperYLimit, lowerYLimit);

        % Display bounding boxes and labels
        displayLargestAreas(masks, regionProps, largestAreasForColors);
        hold off;
    end

    function largestAreasForColors = getLargestAreasForColors(masks, B, regionProps, upperYLimit, lowerYLimit)
        % getLargestAreasForColors - Function to identify largest area for each color.

        largestAreasForColors = containers.Map;
        for colorName = fieldnames(masks)'  % Maintain knowledge of the largest area
            color = colorName{1};
            largestArea = 0;
            largestIdx = -1;
            for k = 1:length(B)
                centroid = regionProps(k).Centroid;
                if centroid(2) > upperYLimit && centroid(2) < lowerYLimit && masks.(color)(round(centroid(2)), round(centroid(1)))
                    if regionProps(k).Area > largestArea
                        largestArea = regionProps(k).Area;
                        largestIdx = k;
                    end
                end
            end
            if largestIdx ~= -1
                largestAreasForColors(color) = largestIdx;
            end
        end
        return;
    end

    function displayLargestAreas(masks, regionProps, largestAreasForColors)
        % displayLargestAreas - Function to display bounding boxes and labels.

        for colorName = fieldnames(masks)'
            color = colorName{1};
            if largestAreasForColors.isKey(color)
                idx = largestAreasForColors(color);
                boundingBox = regionProps(idx).BoundingBox;
                rectangle('Position', boundingBox, 'EdgeColor', 'y', 'LineWidth', 2);
                textPosition = [boundingBox(1), boundingBox(2) - 7];
                text(textPosition(1), textPosition(2), upper(color), 'Color', 'y','FontSize', 8);
            end
        end
    end
end
%% Follow Waypoint
function [actuatorCommands, data] = followWaypoint(data, state, LidarScan, gain, fthrust)
if isempty(data.navigation.waypoint)
    % If no waypoints search for next
    actuatorCommands = sweepBoat();
else
    data.firstSweep = true;
    % Check if the current waypoint is reached
    [data, isReached] = updateWaypointStatus(data, state);
    if isReached
        disp('Reached waypoint')
        data.navigation.waypoint = [];
        actuatorCommands = [0.1, -0.1];
    else
        % Calculate control commands based on the current waypoint
        actuatorCommands = calculateControlCommands(data, state, LidarScan, gain, fthrust);

    end
    % Update the visual representation of waypoints
    data = updateWaypointVisualization(data);
end
    function [data, isReached] = updateWaypointStatus(data, state)
        dN = data.navigation.waypoint(1) - state.Pos(1);
        dE = data.navigation.waypoint(2) - state.Pos(2);
        distToWaypoint = sqrt(dN^2 + dE^2);

        isReached = distToWaypoint < data.GPS.CEP + 0.2;
    end

    function data = updateWaypointVisualization(data)
        % Check if there are no waypoints
        if isempty(data.navigation.waypoint)
            % If there are no waypoints, clear existing circle visualization
            if ~isempty(data.debug.circle_handles)
                delete(data.debug.circle_handles);
                data.debug.circle_handles = [];
            end
        else
            figure(2); % Activate figure 2

            % Delete the existing circle if it exists
            if ~isempty(data.debug.circle_handles)
                delete(data.debug.circle_handles);
                data.debug.circle_handles = [];
            end

            % Get the coordinates of the waypoint
            Y = data.navigation.waypoint(1);
            X = data.navigation.waypoint(2);

            % Plot a new circle at the waypoint and store the handle
            h = viscircles([X, Y], 0.5, 'Color', 'red', 'LineWidth', 1);
            data.debug.circle_handles = h(1);
        end
    end

    function actuatorCommands = calculateControlCommands(data, state, LidarScan, gain, fthrust)
        % Recalculate distance to the waypoint
        dN = data.navigation.waypoint(1) - state.Pos(1);
        dE = data.navigation.waypoint(2) - state.Pos(2);
        yawDesired = atan2(dE, dN);

        % Calculate yaw error and apply angle wrapping
        yawError = wrapToPi(yawDesired - state.rot(3));
        utheta = gain * yawError;

        % Limit forward thrust
        if (fthrust + utheta > 1)
            fthrust = 1 - utheta;
        end

        % Check for obstacles and adjust actuator commands accordingly
        actuatorCommands = handleObstacleAvoidance(data, state, LidarScan, fthrust, utheta);
    end

    function yawErrorWrapped = wrapToPi(yawError)
        % Wrap angle error to the range [-pi, pi]
        yawErrorWrapped = mod(yawError + pi, 2 * pi) - pi;
    end

    function actuatorCommands = handleObstacleAvoidance(data, state, LidarScan, fthrust, utheta)
        % LiDAR points processing
        %reference to azimuth and elevation
        azimuth=0:359;
        elevation=-15:2:15;

        %Find all points that are valid and remember the azimuth and elevation
        valid=find(LidarScan~=Inf);
        [row, col]=ind2sub(size(LidarScan),valid);
        % sphericalCoord=[LidarScan(valid) elevation(row) azimuth(col)];

        %Now move from spherical coordinates to cartesian (still in sensor frame)
        RayF=cosd(azimuth(col));
        RayR=sind(azimuth(col));
        RayD=-sind(elevation(row));
        Rays=[RayF; RayR; RayD]; %magnitude is not 1!
        RayMags=sqrt(1+RayD.*RayD); %magnitude of Rays
        unitRays=Rays./RayMags;     %unit vectors now!
        cartCoord=LidarScan(valid)'.*unitRays;

        %Move the points from sensor frame to FRD
        FRD=cartCoord+[0.27 0 -0.34]';

        %Remove some points due to being on the boat (size of boat inflated
        %slightly due to sensor noise/error!)
        removeFRD=FRD(1,:)>=-0.9 & FRD(1,:)<=0.8 & FRD(2,:)>=-0.4 & FRD(2,:)<=0.4;
        FRDPoints=FRD(:,~removeFRD);

        %move remaining points to NED
        R=[cos(state.rot(3)) -sin(state.rot(3)) 0;
            sin(state.rot(3)) cos(state.rot(3)) 0;
            0 0 1];
        NEDPoints=R*FRDPoints+state.Pos'; %each column is now a point

        %Remove points that are at the water line
        notWater=NEDPoints(3,:)<-0.05;
        NEDPoints=NEDPoints(:,notWater);

        %Move back to FRD
        FRDPoints = R'*(NEDPoints-state.Pos');

        % Determine if any points are inside the obstacle zone
        isInside = inpolygon(FRDPoints(1,:), FRDPoints(2,:), ...
            data.collisionAvoidance.DetectionGrid_FRD(:,1), ...
            data.collisionAvoidance.DetectionGrid_FRD(:,2));
        obstacleDetected = any(isInside);

        % Check for and handle obstacle detection
        if obstacleDetected
            averageY = mean(FRDPoints(2, isInside));
            data.collisionAvoidance.activelyAvoiding = true;

            % Determine turn direction based on averageY
            if averageY > 0
                data.collisionAvoidance.turnDirection = 'left';
            else
                data.collisionAvoidance.turnDirection = 'right';
            end

            % Display obstacle detection message
            if strcmp(data.collisionAvoidance.turnDirection, 'left')
                % disp('Obstacle detected on starboard side');
            else
                % disp('Obstacle detected on port side');
            end
        elseif ~obstacleDetected
            data.collisionAvoidance.activelyAvoiding = false;
            data.collisionAvoidance.turnDirection = '';
        end

        % Adjust actuator commands based on the presence of obstacles
        if data.collisionAvoidance.activelyAvoiding && data.collisionAvoidance.on
            turnAmount = 0.6;
            if strcmp(data.collisionAvoidance.turnDirection, 'left')
                actuatorCommands = [-turnAmount, turnAmount];  % Turn left
            else
                actuatorCommands = [turnAmount, -turnAmount];  % Turn right
            end
        else
            actuatorCommands = fthrust + [utheta, -utheta];
        end
    end

    function actuatorCommands = sweepBoat()
        % Generate a random value between 0 and 1
        randomValue = rand;

        % Randomly decide the direction of the turn
        if rand > 0.5
            % Positive direction
            actuatorCommands = [randomValue, -randomValue];
        else
            % Negative direction
            actuatorCommands = [-randomValue, randomValue];
        end
    end
end