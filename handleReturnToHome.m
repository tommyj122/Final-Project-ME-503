function [actuatorCommands, data] = handleReturnToHome(data, state)

switch data.returnHome.currentState
    case 'NAVIGATING_HOME'
        % Navigate towards the starting location
        navigateToStartLocation(data.returnHome.startLocation);
        if isWithinThreshold(data.returnHome.startLocation, data.returnHome.completionThreshold)
            data.returnHome.currentState = 'AT_HOME';
        end

    case 'AT_HOME'
        data.state.current = 'Complete';

    otherwise
        error('Unknown state in return to home process');
end


% Define the functions used in the switch case here, such as:
% - navigateToStartLocation
% - isWithinThreshold
% - awardPoints
% - moveToNextObjective
end