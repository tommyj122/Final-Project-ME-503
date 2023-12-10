function [actuatorCommands, data] = followWaypoint(data, state, FRDPoints, gain, fthrust)
circ=[sind(0:360);cosd(0:360)];
%Determine the distance to waypoint
dN=data.navigation.waypoint(data.navigation.currentWaypointIdx,1) - state.Pos(1);
dE=data.navigation.waypoint(data.navigation.currentWaypointIdx,2) - state.Pos(2);
dist=sqrt(dN*dN+dE*dE);

% check waypoint progression
if(dist < data.GPS.CEP)
    data.navigation.currentWaypointIdx=data.navigation.currentWaypointIdx+1;
    data.navigation.currentPoint.XData=data.navigation.waypoint(data.navigation.currentWaypointIdx,2);
    data.navigation.currentPoint.YData=data.navigation.waypoint(data.navigation.currentWaypointIdx,1);
    data.navigation.CEP.XData = data.navigation.waypoint(data.navigation.currentWaypointIdx,2)+circ(2,:);
    data.navigation.CEP.YData = data.navigation.waypoint(data.navigation.currentWaypointIdx,1)+circ(1,:);
end
if(data.navigation.currentWaypointIdx > size(data.navigation.waypoint,1))
    actuatorCommands=[0 0];
    return;
end

%determine angle and angle error
yawDesired=atan2(dE,dN);

%error calc and angle wrap
yawError=yawDesired-state.rot(3);
while(yawError)>pi
    yawError=yawError-2*pi;
end
while(yawError)<-pi
    yawError=yawError+2*pi;
end
utheta = gain * yawError;

% Determine forward thrust limiting it to ensure the total thrust doesn't exceed 1
if (fthrust + utheta > 1)
    fthrust = 1 - utheta;
end

%% Obstacle Detection
isInside = inpolygon(FRDPoints(1,:), FRDPoints(2,:), data.collisionAvoidance.DetectionGrid_FRD(:,1), data.collisionAvoidance.DetectionGrid_FRD(:,2));
obstacleDetected = any(isInside);

% Determine the side of the obstacle and make a single decision
if obstacleDetected && ~data.collisionAvoidance.activelyAvoiding
    averageY = mean(FRDPoints(2, isInside));
    data.collisionAvoidance.activelyAvoiding = true;
    if averageY > 0
        data.collisionAvoidance.turnDirection = 'left';  % Obstacle on starboard side, turn left
        disp('Obstacle detected on starboard side')
    else
        data.collisionAvoidance.turnDirection = 'right'; % Obstacle on port side, turn right
        disp('Obstacle detected on port side')
    end
elseif ~obstacleDetected
    data.collisionAvoidance.activelyAvoiding = false;
    data.collisionAvoidance.turnDirection = '';
end

% Adjust actuator commands based on obstacle avoidance mode
if data.collisionAvoidance.activelyAvoiding
    turnAmount = 0.6;
    if strcmp(data.collisionAvoidance.turnDirection, 'left')
        actuatorCommands = [1 - turnAmount, 1 + turnAmount];  % Turn left
    else
        actuatorCommands = [1 + turnAmount, 1 - turnAmount];  % Turn right
    end
else 
    actuatorCommands = fthrust + [utheta, -utheta];
end

% Store old values
data.navigation.eTheta = [yawError, data.navigation.eTheta(1)];

end