function [actuatorCommands, data] = handleDocking(data, state)
switch data.docking.currentState
    case 'SEARCHING_DOCK'
        % Detect the docking bay with the active color
        dockDetected = detectDockingBay(data.docking.activeDockColor);
        if dockDetected
            data.docking.currentState = 'ENTERING_DOCK';
        end

    case 'ENTERING_DOCK'
        % Navigate the ASV into the docking bay
        enterDock();
        if isBowCrossedPairOfPosts()
            data.docking.docked = true;
            data.docking.currentState = 'DOCKED';
        end

    case 'DOCKED'
        % Wait for a command to undock or perform docked tasks
        if shouldUndock()  % Condition to start undocking
            data.docking.currentState = 'LEAVING_DOCK';
        end

    case 'LEAVING_DOCK'
        % Execute undocking sequence
        leaveDock();
        if isBowExitedAndSternNotCrossedPosts()
            data.docking.currentState = 'UNDOCKED';
        end

    case 'UNDOCKED'
        data.state.current = 'SpeedGate';

    otherwise
        error('Unknown state in docking process');
end

% Define the functions used in the switch case here, such as:
% - detectDockingBay
% - enterDock
% - isBowCrossedPairOfPosts
% - shouldUndock
% - leaveDock
% - isBowExitedAndSternNotCrossedPosts
% - moveToNextObjective
end