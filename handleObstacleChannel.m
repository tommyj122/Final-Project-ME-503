function data = handleObstacleChannel(data, state, cameraImage)
%Run the Deep Learning detector
[bboxes,scores, label] = detect(data.obstChannel.detector,cameraImage,'Threshold',0.6);

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
data.debug.imshow.CData=cameraImage;

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
    buoyHeightReal = 1.25; % Actual height of the buoy (m)
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
    
    if norm(abs(waypoint_NED - data.navigation.waypoint)) > 2
        % Store waypoint in data structure
        data.navigation.waypoint = waypoint_NED(1:2);
    end
else
    fprintf('No buoy pair detected.\n');
end

if state.rot(3) < 0.001 || state.rot(3) > pi + 0.01
    data.state.current = 'Docking';
end

end