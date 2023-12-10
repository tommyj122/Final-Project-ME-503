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
% This controls obstacle avoidance
config.avoidObstacles = false; % Not yet implemented

% Initialization
% Check if data is empty and initialize if necessary.
if isempty(data)
    data = initializeData(cameraImage, dock);
end

% LiDAR Processing
LiDAR_FRD = processLiDAR(LidarScan);

% FSM for task management, decisions, and waypoint generation
% The FSM controls the ASV's behavior based on the current state.
switch data.state.current
    case 'NavigationChannel'
        % Handle navigation channel task
        if config.enableNavigationChannel
            [actuatorCommands, data] = handleNavigationChannel(data, state);
        else
            data.state.current = 'ObstacleChannel';
            actuatorCommands = [0 0]; % Default actuator commands
        end
    case 'ObstacleChannel'
        % Handle obstacle channel task
        if config.enableObstacleChannel
            [actuatorCommands, data] = handleObstacleChannel(data, state);
        else
            data.state.current = 'ObstacleField';
            actuatorCommands = [0 0];
        end
    case 'ObstacleField'
        % Handle obstacle field task
        if config.enableObstacleField
            [actuatorCommands, data] = handleObstacleField(data, state);
        else
            data.state.current = 'Docking';
            actuatorCommands = [0 0];
        end
    case 'Docking'
        % Handle docking task
        if config.enableDocking
            [actuatorCommands, data] = handleDocking(data, state);
        else
            data.state.current = 'SpeedGate';
            actuatorCommands = [0 0];
        end
    case 'SpeedGate'
        % Handle speed gate task
        if config.enableSpeedGate
            [actuatorCommands, data] = handleSpeedGate(data, state);
        else
            data.state.current = 'ReturnToHome';
            actuatorCommands = [0 0];
        end
    case 'ReturnToHome'
        % Handle return to home task
        if config.enableReturnToHome
            [actuatorCommands, data] = handleReturnToHome(data, state);
        else
            data.state.current = 'Complete';
            actuatorCommands = [0 0];
        end
    case 'Complete'
        % Indicate mission completion
        error('Mission complete');
    otherwise
        % Handle unknown state
        warning('Unknown objective');
        actuatorCommands = [0 0];
end

% Obstacle avoidance detector and actuator commands adjustment
if config.avoidObstacles
    actuatorCommands = avoidObstacles(actuatorCommands, LiDAR_FRD);
end
end
