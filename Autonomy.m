function [actuatorCommands, data] = Autonomy(state, LidarScan, cameraImage, dock, data)
% Autonomous USV actuator control using a Finite State Machine (FSM) approach.
% Authors: Thomas Jones & Charlotte Downs
% Course: ME 503: Unmanned Vehicle Systems, Fall 2023
% Instructor: Dr. Eric Coyle

% Configuration for enabling/disabling objectives
% These flags control which tasks are active in the FSM.
config.enableNavigationChannel = false; % Not yet implemented
config.enableObstacleChannel = false; % Not yet implemented
config.enableObstacleField = false; % Not yet implemented
config.enableDocking = false; % Not yet implemented
config.enableSpeedGate = false; % Not yet implemented
config.enableReturnToHome = false; % Not yet implemented

% Initialization
% Check if data is empty and initialize if necessary.
if isempty(data)
    data = initializeData(cameraImage, dock);
end

% LiDAR Processing
LiDAR_Points = processLiDAR(state, LidarScan, data.LiDAR.mode);

% FSM for task management, decisions, and waypoint generation
% The FSM controls the ASV's behavior based on the current state.
switch data.state.current
    case 'NavigationChannel'
        % Handle navigation channel task
        if config.enableNavigationChannel
            data = handleNavigationChannel(data, state);
        else
            data.state.current = 'ObstacleChannel';
        end
    case 'ObstacleChannel'
        % Handle obstacle channel task
        if config.enableObstacleChannel
            data = handleObstacleChannel(data, state);
        else
            data.state.current = 'ObstacleField';
        end
    case 'ObstacleField'
        % Handle obstacle field task
        if config.enableObstacleField
            data = handleObstacleField(data, state);
        else
            data.state.current = 'Docking';
        end
    case 'Docking'
        % Handle docking task
        data.collisionAvoidance.on = false;
        if config.enableDocking
            data = handleDocking(data, state);
        else
            data.state.current = 'SpeedGate';
            data.collisionAvoidance.on = true;
        end
    case 'SpeedGate'
        % Handle speed gate task
        if config.enableSpeedGate
            data = handleSpeedGate(data, state);
        else
            data.state.current = 'ReturnToHome';
        end
    case 'ReturnToHome'
        % Handle return to home task
        if config.enableReturnToHome
            data = handleReturnToHome(data, state);
        else
            data.state.current = 'unkown';
        end
    case 'Complete'
        % Indicate mission completion
        error('Mission complete');
    otherwise
        % Handle unknown state
        warning('Unknown objective');
end

% Debug waypoints
data.navigation.waypoint = [15,17;5,5;4,20];

% Follow next commanded waypoint with collision avoidance
% followWaypoint(data, state, LiDAR_Points, steering gain, fthrust)
[actuatorCommands, data] = followWaypoint(data, state, LiDAR_Points, 1.2, 0.4);
end
