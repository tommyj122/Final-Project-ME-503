function FRDPoints = processLiDAR(state, LidarScan, mode)
% This code is directly from Solution #8 Mapping, by Dr. Coyle
%reference to azimuth and elevation
azimuth=0:359;
elevation=-15:2:15;

%Find all points that are valid and remember the azimuth and elevation
valid=find(LidarScan~=Inf);
[row, col]=ind2sub(size(LidarScan),valid);
% sphericalCoord=[LidarScan(valid) elevation(row) azimuth(col)];

%Now move from spherical coordinates to cartesian (still in sensor frame)
RayF=cosd(azimuth(col));
RayR=sind(azimuth(col));
RayD=-sind(elevation(row));
Rays=[RayF; RayR; RayD]; %magnitude is not 1!
RayMags=sqrt(1+RayD.*RayD); %magnitude of Rays
unitRays=Rays./RayMags;     %unit vectors now!
cartCoord=LidarScan(valid)'.*unitRays;

%Move the points from sensor frame to FRD
FRD=cartCoord+[0.27 0 -0.34]';

%Remove some points due to being on the boat (size of boat inflated
%slightly due to sensor noise/error!)
removeFRD=FRD(1,:)>=-0.9 & FRD(1,:)<=0.8 & FRD(2,:)>=-0.4 & FRD(2,:)<=0.4;
FRDPoints=FRD(:,~removeFRD);

%move remaining points to NED
R=[cos(state.rot(3)) -sin(state.rot(3)) 0;
    sin(state.rot(3)) cos(state.rot(3)) 0;
    0 0 1];
NEDPoints=R*FRDPoints+state.Pos'; %each column is now a point

%Remove points that are at the water line
notWater=NEDPoints(3,:)<-0.05;
NEDPoints=NEDPoints(:,notWater);

if strcmp(mode, 'FRD')
    %Move back to FRD
    FRDPoints = R'*(NEDPoints-state.Pos');
end
end

