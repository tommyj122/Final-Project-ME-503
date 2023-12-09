function [actuatorCommands, data] = handleNavigationChannel(cameraImage, data, state)
%Run the Deep Learning detector
[bboxes,scores, label] = detect(data.sensor.detector,cameraImage,'Threshold',0.7);

waypoint = data.navigation.waypoint;

%Place annotations on image and get class list
categories=categorical({'Tall_Green','Tall_Red','Tall_White'});
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
data.navChannel.imshow.CData=cameraImage;

%Now that label_num has a list of the actual classes (1=green, 2=red,
%3=white), and bboxes has the box location we can use to find buoy location
greens=label_num == 1;
reds=label_num == 2;
num_green = sum(sum(greens));
num_red = sum(sum(reds));
if(num_red == 1 && num_green == 1)
    disp('Single pair detected')
    %pixel coordinates
    green_idx = find(greens);
    red_idx = find(reds);
    green = bboxes(green_idx,:);
    red = bboxes(red_idx,:);

    %quick check to make sure the bounding boxes don't overlap
    overlap = (red(1) > green(1) && red(1) < (green(1)+green(3))) || ...
        (green(1) > red(1) && green(1) < (red(1)+red(3)));
    if(~overlap)
        %Get vertical location on the sensor in meters
        f=0.00246;
        pt = (464/2 - green(2))*(.0064/464);
        pb = (green(2) + green(4) - 464/2)*(.0064/464);

        %I know that the bottom is always below the center, and top is always
        %above the center because of mounting. Buoy is also 1.25m tall.
        db=1.25/((pt/pb)+1);
        greenx=db*(f/pt);

        %left/right location in image
        py=((green(1)+green(3)/2) - 618/2)*(.00853/618);
        greeny=greenx*(py/f);

        %Get vertical location on the sensor in meters
        pt = (464/2 - red(2))*(.0064/464);
        pb = (red(2) + red(4) - 464/2)*(.0064/464);

        %I know that the bottom is always below the center, and top is always
        %above the center because of mounting. Buoy is also 1.25m tall.
        db=1.25/((pt/pb)+1);
        redx=db*(0.00246/pt);

        %left/right location in image
        py=((red(1)+red(3)/2) - 618/2)*(.00853/618);
        redy=redx*(py/f);

        %get location in robot frame (zero vertical position because we don't
        %care about it)
        green_flu = [greenx+0.27 greeny 0]';
        red_flu =  [redx+0.27 redy 0]';

        %Get rotation matrix
        yaw=state.rot(3);
        R=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0; 0 0 1];

        %global location
        green_NED=R*green_flu + state.Pos';
        red_NED=R*red_flu + state.Pos';

        waypoint = (green_NED(1:2) +red_NED(1:2))/2;
        data.navigation.waypoint = waypoint;
    end
    %Determine the distance to waypoint
    dN=data.navigation.waypoint(1) - state.Pos(1);
    dE=data.navigation.waypoint(2) - state.Pos(2);
    dist=sqrt(dN*dN+dE*dE);

    % check waypoint progression
    if(dist<data.sensor.CEP)
        %empty the waypoint
        data.waypoint=[];
        actuatorCommands=[0.4 0.4];
    else
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

        %constant thrust
        fthrust=0.3;

        %Turning Controller
        KpT=1;
        utheta=KpT*yawError;

        if(fthrust+utheta > 1)
            fthrust=1 - utheta;
        end

        %compute actuator commands
        actuatorCommands=fthrust+[utheta -utheta];
    end
elseif (num_red == 1 || num_green == 1) && isempty(waypoint)
    % Single buoy detected and no current waypoint
    buoy_idx = [];
    if num_red == 1
        buoy_idx = find(reds, 1);
    elseif num_green == 1
        buoy_idx = find(greens, 1);
    end

    if ~isempty(buoy_idx)
        % Extract the coordinates of the buoy
        buoy_bbox = bboxes(buoy_idx, :);

        % Determine if the buoy is to the left or right of the center of the image
        image_center_x = size(cameraImage, 2) / 2; % Horizontal center of the image
        buoy_center_x = buoy_bbox(1) + buoy_bbox(3) / 2; % Horizontal center of the buoy

        % Turning direction: turn right if buoy is to the left, turn left if buoy is to the right
        if buoy_center_x < image_center_x
            % Buoy is to the left, turn right
            actuatorCommands = [-0.1, 0.1];
        else
            % Buoy is to the right, turn left
            actuatorCommands = [0.1, -0.1];
        end

        disp('No pair detected, turning towards single detected buoy');
    end
else
    % No buoys detected
    if isempty(waypoint)
        disp('No buoys detected, no waypoint set');
        % Set default actuator commands in case no waypoint is set and no buoys detected
        actuatorCommands = [-0.3, 0.3];
    else
        % Continue towards last known waypoint
        disp('No buoys detected, following last waypoint');
        %Determine the distance to waypoint
        dN=data.navigation.waypoint(1) - state.Pos(1);
        dE=data.navigation.waypoint(2) - state.Pos(2);
        dist=sqrt(dN*dN+dE*dE);

        % check waypoint progression
        if(dist<data.sensor.CEP)
            %empty the waypoint
            data.waypoint=[];
            actuatorCommands=[0.4 0.4];
        else
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

            %constant thrust
            fthrust=0.3;

            %Turning Controller
            KpT=1;
            utheta=KpT*yawError;

            if(fthrust+utheta > 1)
                fthrust=1 - utheta;
            end

            %compute actuator commands
            actuatorCommands=fthrust+[utheta -utheta];
        end
    end
end
end