function [actuatorCommands, data] = Autonomy(state, LidarScan, cameraImage, dock, data)
% Autonomous USV actuator control using a Finite State Machine (FSM) approach.
% Authors: Thomas Jones & Charlotte Downs
% Course: ME 503: Unmanned Vehicle Systems, Fall 2023
% Instructor: Dr. Eric Coyle

% Configuration for enabling/disabling objectives
% These flags control which tasks are active in the FSM.
debugMode = true; % Activate debug mode

config.enableNavigationChannel = false; % Fully implemented, lightly tested (Thomas)

config.enableObstacleChannel = false; % Fully implemented, lightly tested need to fix DNN (Thomas)

config.enableObstacleField = false; % Not to be attempted

config.enableDocking = true; % Work in progress (Charlotte)

config.enableSpeedGate = false; % Not yet implemented (Thomas)

config.enableReturnToHome = false; % Not yet implemented (Charlotte)

% Initialization
% Check if data is empty and initialize if necessary.
if isempty(data)
    data = initializeData(state, cameraImage, dock, debugMode);
end

% LiDAR Processing
LiDAR_Points = processLiDAR(state, LidarScan, data.LiDAR.mode);

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
        data.collisionAvoidance.on = false;
        if config.enableDocking
            data = handleDocking(data, state);
        else
            data.state.current = 'SpeedGate';
            data.state.previous = 'Docking';
            data.collisionAvoidance.on = true;
        end
    case 'SpeedGate'
        % Handle speed gate task
        if config.enableSpeedGate
            data = handleSpeedGate(data, state);
        else
            data.state.current = 'ReturnToHome';
            data.state.previous = 'SpeedGate';
        end
    case 'ReturnToHome'
        % Handle return to home task
        if config.enableReturnToHome
            data = handleReturnToHome(data, state);
        else
            data.state.current = 'Complete';
            data.state.previous = 'ReturnToHome';
        end
    case 'Complete'
        % Indicate mission completion
        error('Mission complete');
    otherwise
        % Handle debug state
        warning('debug objective');
end

% Follow next commanded waypoint with collision avoidance
% followWaypoint(data, state, LiDAR_Points, steering gain, fthrust)
[actuatorCommands, data] = followWaypoint(data, state, LiDAR_Points,...
    data.actuator.steeringGain, data.actuator.fthrust);
end
