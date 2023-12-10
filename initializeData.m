function data = initializeData(cameraImage, dock)
% Initialize state variables and data structures for the Autonomy.m function.

% Debugging and Logging
data.debug = struct();
data.debug.mode = false; % Debug mode flag

% State Flags
data.state = struct();
data.state.previous = 'safeMode'; % Previous state
data.state.current = 'NavigationChannel'; % Current state

% Sensor Data
data.sensor = struct();
data.sensor.CEP = 0.120; % Circular Error Probable from GPS reading

% Waypoint and Navigation Data
data.navigation = struct();
data.navigation.requireLidar = false; % Flag to check if LiDAR is required
data.navigation.requireCamera = false; % Flag to check if camera is required
data.navigation.waypoint = []; % Current waypoint
data.navigation.currentWaypointIdx = 1; % Index of current waypoint
% Navigation grid setup
data.navigation.grid = struct();
data.navigation.grid.cellSize = 0.2; % Size of each grid cell in meters
data.navigation.grid.northBound = [-5, 45]; % North boundary of the grid
data.navigation.grid.eastBound = [-5, 85]; % East boundary of the grid
% Calculate grid size based on boundaries and cell size
data.navigation.grid.sizeN = (data.navigation.grid.northBound(2) - data.navigation.grid.northBound(1)) / data.navigation.grid.cellSize;
data.navigation.grid.sizeE = (data.navigation.grid.eastBound(2) - data.navigation.grid.eastBound(1)) / data.navigation.grid.cellSize;

% Actuator Status and Control
data.actuator = struct();
data.actuator.fthrust = 0.4; % Forward thrust
data.actuator.turnAmount = 0.5; % Turn amount

% Collision Avoidance
data.collisionAvoidance = struct();
data.collisionAvoidance.safeDistance = 0.35; % Safe distance in meters
data.collisionAvoidance.chosenTurnDirection = ''; % Chosen direction for avoidance maneuver
% LiDAR sensor placement transformation
DetectionGrid_sensor = [0, 0; 0.2, 0.55; 1.3, 0.55; 1.3, -0.55; 0.2, -0.55; 0, 0];
trnsfLiDAR2Origin = [0.27, 0];
data.collisionAvoidance.DetectionGrid_FRD = DetectionGrid_sensor + trnsfLiDAR2Origin;

% Navigation Channel
data.navChannel = struct();
load('NavChannelYOLO', 'detector', 'info'); % Large buoy detector
data.navChannel.detector = detector; % Object detection neural network
data.navChannel.info = info; % Information about the detector
% Debug mode for displaying images
if data.debug.mode
    figure;
    data.navChannel.imshow = imshow(cameraImage);
end
data.navChannel.status = 'SEARCHING_GATE_1'; % Status of navigation channel task
data.navChannel.gate1Waypoint = []; % Waypoint for gate 1
data.navChannel.gate2Waypoint = []; % Waypoint for gate 2

% Obstacle Channel
data.obstChannel.status = 'SEARCHING'; % Status of obstacle channel task
data.obstChannel.currentGate = 1; % Current gate being navigated
data.obstChannel.gateWaypoints = cell(1, 15); % Waypoints for each of the 15 gates

% Obstacle Field
data.obstField.currentState = 'SEARCHING_ENTRY'; % Current state in obstacle field task
data.obstField.centralBuoyCircumnavigated = false; % Flag for buoy circumnavigation
data.obstField.obstacleFieldEntered = false; % Flag for entering obstacle field
data.obstField.openingLocation = []; % Location of opening in the obstacle field
data.obstField.whiteBuoyLocation = []; % Location of the white buoy
data.obstField.exitLocation = []; % Exit location from the obstacle field

% Docking
data.docking.currentState = 'SEARCHING_DOCK'; % Current state in docking task
data.docking.activeDockColor = dock; % Active dock color
data.docking.docked = false; % Docking status

% Speed Gate
data.speedGate.currentState = 'SEARCHING_GATE_ENTRY'; % Current state in speed gate task
data.speedGate.timerStarted = false; % Flag for timer status
data.speedGate.markBuoyCircumnavigated = false; % Flag for mark buoy circumnavigation

% Return to Home
data.returnHome.currentState = 'NAVIGATING_HOME'; % Current state in return to home task
data.returnHome.distanceTraveled = 0; % Total distance traveled
data.returnHome.startLocation = [0, 0]; % Replace with actual start coordinates
data.returnHome.completionThreshold = 2; % Threshold for completion in meters

end
