function [actuatorCommands, data] = followWaypoint(data, state, FRDPoints, gain, fthrust)
% Check if there are any waypoints to follow
if isempty(data.navigation.waypoint)
    % If no waypoints, set default actuator commands
    actuatorCommands = [0.1, 0.1];
else
    % Check if the current waypoint is reached
    data = updateWaypointStatus(data, state);

    % Update the visual representation of waypoints
    data = updateWaypointVisualization(data);

    % Calculate control commands based on the current waypoint
    actuatorCommands = calculateControlCommands(data, state, FRDPoints, gain, fthrust);
end
end

function [data, isReached] = updateWaypointStatus(data, state)
dN = data.navigation.waypoint(1) - state.Pos(1);
dE = data.navigation.waypoint(2) - state.Pos(2);
distToWaypoint = sqrt(dN^2 + dE^2);

isReached = distToWaypoint < data.GPS.CEP;
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


function actuatorCommands = calculateControlCommands(data, state, FRDPoints, gain, fthrust)
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
actuatorCommands = handleObstacleAvoidance(data, FRDPoints, fthrust, utheta);
end

function yawErrorWrapped = wrapToPi(yawError)
% Wrap angle error to the range [-pi, pi]
yawErrorWrapped = mod(yawError + pi, 2 * pi) - pi;
end

function actuatorCommands = handleObstacleAvoidance(data, FRDPoints, fthrust, utheta)
isInside = inpolygon(FRDPoints(1,:), FRDPoints(2,:), ...
    data.collisionAvoidance.DetectionGrid_FRD(:,1), ...
    data.collisionAvoidance.DetectionGrid_FRD(:,2));
obstacleDetected = any(isInside);

% Check for and handle obstacle detection
if obstacleDetected && ~data.collisionAvoidance.activelyAvoiding
    averageY = mean(FRDPoints(2, isInside));
    data.collisionAvoidance.activelyAvoiding = true;
    data.collisionAvoidance.turnDirection = determineTurnDirection(averageY);
    dispObstacleDetection(data.collisionAvoidance.turnDirection);
elseif ~obstacleDetected
    data.collisionAvoidance.activelyAvoiding = false;
    data.collisionAvoidance.turnDirection = '';
end

% Adjust actuator commands based on the presence of obstacles
if data.collisionAvoidance.activelyAvoiding && data.collisionAvoidance.on
    turnAmount = 0.6;
    actuatorCommands = adjustForObstacle(data.collisionAvoidance.turnDirection, turnAmount);
else
    actuatorCommands = fthrust + [utheta, -utheta];
end
end

function turnDirection = determineTurnDirection(averageY)
if averageY > 0
    turnDirection = 'left';
else
    turnDirection = 'right';
end
end

function dispObstacleDetection(turnDirection)
if strcmp(turnDirection, 'left')
    disp('Obstacle detected on starboard side');
else
    disp('Obstacle detected on port side');
end
end

function actuatorCommands = adjustForObstacle(turnDirection, turnAmount)
if strcmp(turnDirection, 'left')
    actuatorCommands = [1 - turnAmount, 1 + turnAmount]; % Turn left
else
    actuatorCommands = [1 + turnAmount, 1 - turnAmount]; % Turn right
end
end
