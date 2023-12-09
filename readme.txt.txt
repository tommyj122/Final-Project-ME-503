%Inputs:
%state - a structure containing the vehicle state
%       state.time - time in seconds
%       state.Pos - NED location in meters [N,E,D]
%       state.Vel - velocity in meters/sec [N_dot,E_dot,D_dot]
%       state.rot - angular position in radians [roll, pitch, yaw]
%       state.rVel - angular velocity in rad/s [roll, pitch, yaw]
%LidarScan - distances measured from the last Lidar scan. Each row is an
%       individual laser, each column is a different azimuth angle.
%cameraImage - A RGB picture as seen by the camera
%data - a structure that used to pass data from one iteration to the next.
%       The only data here is what you previously assigned.
%
%Outputs:
%actuatorInputs - a variable containing all actuator commands
%data - a structure that used to pass data from one iteration to the next.