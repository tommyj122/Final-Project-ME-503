function [actuatorCommands, data] = handleNavigationChannel(cameraImage, data)
% Detect buoys in view
[bboxes, scores, label] = detect(data.sensor.detector, cameraImage, 'Threshold', 0.6);

% Place annotations on image and get class list
categories = categorical({'Tall_Green', 'Tall_Red', 'Tall_White'});
colorSpec = [0 1 0; 1 0 0; 1 1 1; 1 1 0; 0 0 1];
label_num = zeros(length(scores), 1);
for k = 1:length(scores)
    minIdx = find(label(k) == categories);
    label_num(k) = minIdx;

    annotation = sprintf('%s %4.2f', label(k), scores(k));
    cameraImage = insertObjectAnnotation(cameraImage, 'rectangle', bboxes(k, :),...
        annotation, 'FontSize', 10, 'Color', uint8(255 * colorSpec(minIdx, :)));
end
if data.debug.mode == true
    data.imshow.CData = cameraImage;
end

% Find indices of green and red buoys
greenIdx = find(label_num == 1); % Assuming 1 corresponds to 'Tall_Green'
redIdx = find(label_num == 2);   % Assuming 2 corresponds to 'Tall_Red'

% % Navigational decision based on buoy visibility
% if length(greenIdx) + length(redIdx) >= 4 % All buoys visible
%     % Implement logic for navigating through the closest pair
%     % ...
%     % Determine waypoints and generate actuator commands
%     actuatorCommands = ...; % Fill in with your command logic
% elseif length(greenIdx) + length(redIdx) > 0 % Partial visibility
%     % Implement logic for navigating to or identifying the closest buoy pair
%     % ...
%     % Determine waypoints and generate actuator commands
%     actuatorCommands = ...; % Fill in with your command logic
% else
%     % Default behavior if no buoys are detected
%     actuatorCommands = [1 -1];
% end

% Update FSM state if needed
% data.state.current = updateFSMState(data, ...); % Other parameters as needed

actuatorCommands = [0 0];
end