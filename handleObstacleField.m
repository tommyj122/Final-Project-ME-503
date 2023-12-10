function [actuatorCommands, data] = handleObstacleField(data, state)
switch data.obstField.currentState
    case 'SEARCHING_ENTRY'
        % Detect opening in the obstacle field
        data.obstField.openingLocation = detectOpeningInObstacleField();
        if ~isempty(data.obstField.openingLocation)
            data.obstField.currentState = 'NAVIGATING_TO_FIELD';
        end

    case 'NAVIGATING_TO_FIELD'
        % Navigate to the detected opening
        navigateToWaypoint(data.obstField.openingLocation, true);  % true for obstacle avoidance
        if arrivedAtWaypoint(data.obstField.openingLocation)
            data.obstField.obstacleFieldEntered = true;
            data.obstField.currentState = 'SEARCHING_WHITE_BUOY';
        end

    case 'SEARCHING_WHITE_BUOY'
        % Detect the central white buoy
        data.obstField.whiteBuoyLocation = detectWhiteBuoy();
        if ~isempty(data.obstField.whiteBuoyLocation)
            data.obstField.currentState = 'CIRCLING_WHITE_BUOY';
        end

    case 'CIRCLING_WHITE_BUOY'
        % Circle the white buoy
        if ~data.obstField.centralBuoyCircumnavigated
            data.obstField.centralBuoyCircumnavigated = circleBuoy(data.obstField.whiteBuoyLocation);
        else
            data.obstField.currentState = 'EXITING_FIELD';
        end

    case 'EXITING_FIELD'
        % Navigate out of the obstacle field
        data.obstField.exitLocation = findExitLocation();
        navigateToWaypoint(data.obstField.exitLocation, true);
        if arrivedAtWaypoint(data.obstField.exitLocation)
            data.state.current = 'docking';
        end

    otherwise
        error('Unknown state in obstacle field process');
end

% Define the functions used in the switch case here, such as:
% - detectOpeningInObstacleField
% - navigateToWaypoint
% - arrivedAtWaypoint
% - detectWhiteBuoy
% - circleBuoy
% - findExitLocation
% - moveToNextObjective
end