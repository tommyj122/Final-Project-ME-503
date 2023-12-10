function [actuatorCommands, data] = handleObstacleChannel(data, state)
while data.obstChannel.currentGate <= numGates
    switch data.obstChannel.status
        case 'SEARCHING'
            % Detect buoys for the current gate
            buoys = detectBuoysForGate(data.obstChannel.currentGate);
            if size(buoys, 1) >= 2  % Check if at least 2 buoys are detected
                data.obstChannel.gateWaypoints{data.obstChannel.currentGate} = calculateMidpoint(buoys(1, :), buoys(2, :));
                data.obstChannel.status = 'NAVIGATING';
            end

        case 'NAVIGATING'
            waypoint = data.obstChannel.gateWaypoints{data.obstChannel.currentGate};
            if isempty(waypoint)
                error(['Waypoint for Gate ', num2str(data.obstChannel.currentGate), ' not set']);
            end
            navigateToWaypoint(waypoint, true);  % true for obstacle avoidance
            if arrivedAtWaypoint(waypoint)
                data.obstChannel.status = 'SEARCHING';  % Reset state for next gate
                data.obstChannel.currentGate = data.obstChannel.currentGate + 1;  % Move to the next gate
            end

        otherwise
            error('Unknown state in obstacle channel process');
    end
end

% Complete all gates, move to next objective
data.state.current = 'ObstacleField';

% Define the functions used in the switch case here, such as:
% - detectBuoysForGate
% - calculateMidpoint
% - navigateToWaypoint
% - arrivedAtWaypoint
% - moveToNextObjective
end