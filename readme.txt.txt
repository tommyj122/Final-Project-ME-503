function data = handleNavigationChannel(data, state, cameraImage)
switch data.navChannel.status
    case {'SEARCHING_GATE_1', 'SEARCHING_GATE_2'}
        data = searchGate(data, state, cameraImage);
    case {'NAVIGATING_GATE_1', 'NAVIGATING_GATE_2'}
        data = navigateGate(data, state);
    case 'MOVING_TO_NEXT_OBJECTIVE'
        data = moveToNextObjective(data);
    otherwise
        disp(data.navChannel.status)
        error('Unknown state in navigation channel process');
end
end

function data = searchGate(data, state, cameraImage)
    gateNum = str2double(regexp(data.navChannel.status, '\d', 'match'));
    [data, gateSeen] = detectBuoysWithinRange(data, state, cameraImage, data.navChannel.range);

    if gateSeen && ~isempty(data.navChannel.buoyGreenWaypoint) && ~isempty(data.navChannel.buoyRedWaypoint)
        greenN = data.navChannel.buoyGreenWaypoint(1);
        greenE = data.navChannel.buoyGreenWaypoint(2);
        redN = data.navChannel.buoyRedWaypoint(1);
        redE = data.navChannel.buoyRedWaypoint(2);

        meanN = mean([greenN, redN]);
        meanE = mean([greenE, redE]);

        data.navigation.waypoint = [meanN; meanE];
        disp(data.navigation.waypoint)
        data.navChannel.status = sprintf('NAVIGATING_GATE_%d', gateNum);
        data.state.searchMode = false;
        debugMessage(data, sprintf('Found gate %d!', gateNum));
    else
        data.state.searchMode = true;
        debugMessage(data, sprintf('Searching gate %d...', gateNum));
    end
end

function data = navigateGate(data, state)
gateNum = str2double(regexp(data.navChannel.status, '\d', 'match'));
if isempty(data.navigation.waypoint)
    error(sprintf('Gate %d waypoint not set', gateNum));
end
dN = data.navigation.waypoint(1) - state.Pos(1);
dE = data.navigation.waypoint(2) - state.Pos(2);
dist = sqrt(dN^2 + dE^2);
if (dist < data.GPS.CEP) && gateNum < 2
    data.navChannel.status = sprintf('SEARCHING_GATE_%d', gateNum + 1);
elseif (dist < data.GPS.CEP) && gateNum > 2
    data.navChannel.status = 'MOVING_TO_NEXT_OBJECTIVE';
end
    debugMessage(data, sprintf('Navigating gate %d...', gateNum));
end

function data = moveToNextObjective(data)
debugMessage(data, 'Navigation Channel Complete!');
data.state.current = 'ObstacleChannel';
end

function debugMessage(data, message)
if data.debug.mode
    disp(message);
end
end

function [data, gateSeen] = detectBuoysWithinRange(data, state, cameraImage, range)
% Run the Deep Learning detector
[bboxes, scores, labels] = detect(data.navChannel.detector, cameraImage, 'Threshold', data.navChannel.detectorThreshold);

% Filter out overlapping bounding boxes
[bboxes, scores, labels] = filterOverlappingBoxes(bboxes, scores, labels);

% Process detections
[data, greens, reds] = processDetections(data, state, cameraImage, bboxes, scores, labels, range);

% Check if only one green and one red buoy are present
gateSeen = (sum(greens) == 1) && (sum(reds) == 1);
end

function [bboxes, scores, labels] = filterOverlappingBoxes(bboxes, scores, labels)
n = size(bboxes, 1);
overlapRatio = bboxOverlapRatio(bboxes, bboxes);
overlapRatio(1:n+1:end) = 0; % Ignore self-overlap
index = sum(overlapRatio > 0.2, 2) == 0;
bboxes = bboxes(index, :);
scores = scores(index);
labels = labels(index);
end

function [data, greens, reds] = processDetections(data, state, cameraImage, bboxes, scores, labels, range)
categories = categorical({'Tall_Green', 'Tall_Red', 'Tall_White'});
colorSpec = [0 1 0; 1 0 0; 1 1 1];
data.navChannel.validBuoys = struct('bbox', {}, 'label', {}, 'score', {}, 'buoyNED', {}, 'distanceFRD', {});
greens = false(length(scores), 1);
reds = false(length(scores), 1);

for k = 1:length(scores)
    minIdx = find(labels(k) == categories);
    annotation = sprintf('%s %4.2f', labels(k), scores(k));
    cameraImage = insertObjectAnnotation(cameraImage, 'rectangle', bboxes(k, :), annotation, 'FontSize', 10, 'Color', uint8(255 * colorSpec(minIdx, :)));
    [distanceFRD, buoyNED] = calculateBuoyNED(bboxes(k, :), state);

    if isWithinRange(distanceFRD, range)
        data.navChannel.validBuoys(end + 1) = createBuoyStruct(bboxes(k, :), string(labels(k)), scores(k), buoyNED, distanceFRD);
        [data, greens, reds] = updateWaypoints(data, string(labels(k)), buoyNED, greens, reds, k);
    end
end
data.debug.imshow.CData = cameraImage;
end

function flag = isWithinRange(distance, range)
flag = (distance >= range(1)) && (distance <= range(2));
end

function buoyStruct = createBuoyStruct(bbox, label, score, buoyNED, distanceFRD)
buoyStruct = struct('bbox', bbox, 'label', label, 'score', score, 'buoyNED', buoyNED, 'distanceFRD', distanceFRD);
end

function [data, greens, reds] = updateWaypoints(data, label, buoyNED, greens, reds, idx)
if strcmp(label, 'Tall_Green')
    data.navChannel.buoyGreenWaypoint = buoyNED;
    greens(idx) = true;
elseif strcmp(label, 'Tall_Red')
    data.navChannel.buoyRedWaypoint = buoyNED;
    reds(idx) = true;
end
end

function [distanceFRD, buoyNED] = calculateBuoyNED(bbox, state)
% Constants
f = 0.00246;
height = 1.25; % Buoy height in meters

% Get vertical location on the sensor in meters
pt = (464 / 2 - bbox(2)) * (0.0064 / 464);
pb = (bbox(2) + bbox(4) - 464 / 2) * (0.0064 / 464);

% Calculate distances
db = height / ((pt / pb) + 1);
x = db * (f / pt);

% Left/right location in image
py = ((bbox(1) + bbox(3) / 2) - 618 / 2) * (0.00853 / 618);
y = x * (py / f);

% Assign calculated values to flu
flu = [x + 0.27, y, 0]';

% Convert flu to FRD (Forward-Right-Down)
frd = [flu(1), -flu(2), -flu(3)];

% Calculate the distance in FRD
distanceFRD = norm(frd);

% Get rotation matrix
yaw = state.rot(3);
R = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];

% Calculate Global location (NED)
buoyNED = R * flu + state.Pos';
end