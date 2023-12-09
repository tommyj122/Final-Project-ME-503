function [actuatorCommands, data] = Autonomy(state, LidarScan, cameraImage, dock, data)
%% Intro & Information
% Autonomous USV actuator control using a Finite State Machine (FSM) approach.
% Thomas Jones & Charlotte Downs
% ME 503: Unmanned Vehicle Systems
% Fall 2023, Dr. Eric Coyle
%Inputs:
%state - a structure containing the vehicle state
%       state.time - time in seconds
%       state.Pos - NED location in meters [N,E,D]
%       state.Vel - velocity in meters/sec [N_dot,E_dot,D_dot]
%       state.rot - angular position in radians [roll, pitch, yaw]
%       state.rVel - angular velocity in rad/s [roll, pitch, yaw]
%LidarScan - distances measured from the last Lidar scan. Each row is an
%       individual laser, each column is a different azimuth angle.
%cameraImage - A RGB picture as seen by the camera
%data - a structure that used to pass data from one iteration to the next.
%       The only data here is what you previously assigned.
%
%Outputs:
%actuatorInputs - a variable containing all actuator commands
%data - a structure that used to pass data from one iteration to the next.
%% Main Autonomy Loop
% Initialization
if isempty(data)
    data = initializeData(cameraImage);
end

% FSM for task management, decisions, and waypoint generation
switch data.state.current
    case 'NavigationChannel'
        [actuatorCommands, data] = handleNavigationChannel(cameraImage, data);
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

actuatorCommands = [0 0];
end
