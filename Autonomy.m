function [actuatorCommands, data] = Autonomy(state, LidarScan, cameraImage, dock, data)
% Autonomous USV actuator control using a Finite State Machine (FSM) approach.
% Thomas Jones & Charlotte Downs
% ME 503: Unmanned Vehicle Systems
% Fall 2023, Dr. Eric Coyle

% Initialization
if isempty(data)
    data = initializeData(cameraImage);
end

% FSM for task management, decisions, and waypoint generation
switch data.state.current
    case 'NavigationChannel'
        [actuatorCommands, data] = handleNavigationChannel(cameraImage, data, state);
    case 'ObstacleChannel'
        [actuatorCommands, data] = handleObstacleChannel(LidarScan, cameraImage, data);
    case 'ObstacleField'
        [actuatorCommands, data] = handleObstacleField(LidarScan, cameraImage, data);
    case 'Docking'
        [actuatorCommands, data] = handleDocking(LidarScan, cameraImage, dock, data);
    case 'SpeedGate'
        [actuatorCommands, data] = handleSpeedGate(LidarScan, cameraImage, data);
    case 'ReturnToHome'
        [actuatorCommands, data] = handleReturnToHome(LidarScan, cameraImage, data);
    otherwise
        [actuatorCommands, data] = handleDefault(LidarScan, cameraImage, data);
end
end
