function data = initializeData(cameraImage)
% Initialize Autonomy.m and state variables
% State Flags
data.state = struct();
data.state.previous = 'safeMode';
data.state.current = 'NavigationChannel';

% Sensor Data
data.sensor = struct();
data.sensor.CEP = 0.120; % Circular Error Probable from GPS reading
load('BoatYoloNetwork','detector','info');
data.sensor.detector=detector;
data.sensor.info=info;

% Waypoint and Navigation Data
data.navigation = struct();
data.navigation.requireLidar = false;
data.navigation.requireCamera = false;
data.navigation.waypoint = [];
data.navigation.currentWaypointIdx = 1;
data.navigation.grid = struct();
data.navigation.grid.cellSize = 0.2;
data.navigation.grid.northBound = [-5, 45];
data.navigation.grid.eastBound = [-5, 85];
data.navigation.grid.sizeN = (data.navigation.grid.northBound(2) - data.navigation.grid.northBound(1)) / data.navigation.grid.cellSize;
data.navigation.grid.sizeE = (data.navigation.grid.eastBound(2) - data.navigation.grid.eastBound(1)) / data.navigation.grid.cellSize;

% Actuator Status and Control
data.actuator = struct();
data.actuator.fthrust = 0.4; % Forward thrust
data.actuator.turnAmount = 0.5; % Turn amount

% Debugging and Logging
data.debug = struct();
data.debug.mode = true;

% Collision Avoidance
data.collisionAvoidance = struct();
data.collisionAvoidance.safeDistance = 0.35; % m
data.collisionAvoidance.chosenTurnDirection = '';
DetectionGrid_sensor = [0,0;0.2,0.55;1.3,0.55;1.3,-0.55;0.2,-0.55;0,0];
trnsfLiDAR2Origin = [0.27,0];
data.collisionAvoidance.DetectionGrid_FRD = DetectionGrid_sensor + trnsfLiDAR2Origin;

% Navigation Channel
data.navChannel = struct();
if data.debug.mode == true
    figure;
    data.navChannel.imshow=imshow(cameraImage);
end
data.navChannel.gate_status = 'first_gate_not_passed';

buoyStruct = struct('Location', [], 'Gate', '', 'Color', '');
data.navChannel.buoyArray = repmat(buoyStruct, 1, 2);

end