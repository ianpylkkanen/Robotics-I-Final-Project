% GENERATION OF WAYPOINTS
% 
% Adopted from: Mathworks Inc.
% 
% Lachlan Moore
% Ian Pylkkanen
% Sudarshan Thirmualai
% 
% December 2020

%% Common parameters
% NOTE: waypoints, waypointTimes, and trajTimes MUST be the same number of columns

% Rigid Body Tree information using generic robot to define home configuration
load gen3
load gen3positions
eeName = 'Gripper';
numJoints = numel(gen3.homeConfiguration);
ikInitGuess = gen3.homeConfiguration;

% Maximum number of waypoints (for Simulink)
maxWaypoints = 20;

% Positions (X Y Z)
waypoints = [-475 -400 80 ; 475 -400 80; 475 -400 1000; 750 1800 750; 750 1900 750; 750 1800 750; -750 1800 750; -750 1900 750; -750 1800 750; 0 1800 0; 0 1900 0; 0 1800 0; -961 852 -58]';
         
% Euler Angles (Z Y X) relative to the home orientation       
orientations = [0     0    0;
                pi/8  0    0; 
                0     0   -pi/2;
                0     0   -pi/2;
                0     0    0]';   
            
% Array of waypoint times
waypointTimes = 0:2:24;

% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      0  1  0;
                      0  0  0;
                      0  1  0;
                      0  1  0;
                      0  0  0;
                      1  0  0;
                      1  0  0;
                      0  0  0;
                      0  1  0
                      0  1  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
