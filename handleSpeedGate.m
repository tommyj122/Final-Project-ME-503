function [actuatorCommands, data] = handleSpeedGate(data, state)
switch data.speedGate.currentState
    case 'SEARCHING_GATE_ENTRY'
        % Detect the gate buoys
        gateDetected = detectGateBuoys();
        if gateDetected
            data.speedGate.currentState = 'ENTERING_GATE';
        end

    case 'ENTERING_GATE'
        % Navigate through the gate buoys
        enterGate();
        if isOriginCrossedGate()
            data.speedGate.timerStarted = true;
            data.speedGate.currentState = 'CIRCLING_MARK_BUOY';
        end

    case 'CIRCLING_MARK_BUOY'
        % Circle the mark buoy
        if ~data.speedGate.markBuoyCircumnavigated
            data.speedGate.markBuoyCircumnavigated = circleMarkBuoy();
        else
            data.speedGate.currentState = 'EXITING_GATE';
        end

    case 'EXITING_GATE'
        % Exit through the gate buoys
        exitGate();
        if isSternCrossedGate()
            stopTimer();  % Stop the timing for the challenge
            data.speedGate.currentState = 'COMPLETED';
        end

    case 'COMPLETED'
        data.state.current = 'ReturnToHome';
    otherwise
        error('Unknown state in speed gate process');
end

% Define the functions used in the switch case here, such as:
% - detectGateBuoys
% - enterGate
% - isOriginCrossedGate
% - circleMarkBuoy
% - exitGate
% - isSternCrossedGate
% - stopTimer
% - moveToNextObjective
end