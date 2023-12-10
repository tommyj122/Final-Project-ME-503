function [actuatorCommands, data] = handleNavigationChannel(data, state)
switch data.navChannel.search_status
    case 'SEARCHING_GATE_1'
        buoys = detectBuoysWithinRange(8);  % Detect buoys within 8 meters
        if size(buoys, 1) >= 2  % Check if at least 2 buoys are detected
            data.navChannel.gate1Waypoint = calculateMidpoint(buoys(1, :), buoys(2, :));
            data.navChannel.search_status = 'NAVIGATING_GATE_1';
        end

    case 'NAVIGATING_GATE_1'
        if isempty(data.navChannel.gate1Waypoint)
            error('Gate 1 waypoint not set');
        end
        navigateToWaypoint(data.navChannel.gate1Waypoint, true);  % true for obstacle avoidance
        if arrivedAtWaypoint(gate1Waypoint)
            data.navChannel.search_status = 'SEARCHING_GATE_2';
        end

    case 'SEARCHING_GATE_2'
        buoys = detectBuoysWithinRange(8); % minimum 2 meters?
        if size(buoys, 1) >= 2
            data.navChannel.gate2Waypoint = calculateMidpoint(buoys(1, :), buoys(2, :));
            data.navChannel.search_status = 'NAVIGATING_GATE_2';
        end

    case 'NAVIGATING_GATE_2'
        if isempty(data.navChannel.gate2Waypoint)
            error('Gate 2 waypoint not set');
        end
        navigateToWaypoint(data.navChannel.gate2Waypoint, true);
        if arrivedAtWaypoint(data.navChannel.gate2Waypoint)
            data.navChannel.search_status = 'MOVING_TO_NEXT_OBJECTIVE';
        end

    case 'MOVING_TO_NEXT_OBJECTIVE'
        data.state.current = 'ObstacleChannel';
    otherwise
        error('Unknown state in navigation channel process');
end

% Define the functions used in the switch case here, such as:
% - detectBuoysWithinRange
% - calculateMidpoint
% - navigateToWaypoint
% - arrivedAtWaypoint
end