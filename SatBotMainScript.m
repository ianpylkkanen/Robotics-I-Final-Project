% MAIN SCRIPT FOR ROBOT DEFINITION, VISION SIMULATION, PATH PLANNING, AND TRAJECTORY GENERATION
% 
% Adopted from: Mathworks Inc.
% 
% Lachlan Moore
% Ian Pylkkanen
% Sudarshan Thirumalai
% 
% December 2020

%% Setup
clear, clc, close all

% Define waypoint information
createWaypointData;
waypoints = waypoints;
jointAnglesHome = [0; 0; 0; 0; 0; 0; 0];

%% Definition of POE Parameters
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];
zv = [0; 0; 0];

H = [ez, ey, ez, ez, ex, ez, ex];
P = [400*ez, 320*ey - 720*ex, -720*ey, zv, 720*ex, -320*ez, -475*ex, zv];
type = [0, 0, 0, 0, 0, 0, 0];
n = 7;

myrobot.H = H;
myrobot.P = P;
myrobot.joint_type = type;

[robot,colLink]=collisionBody(myrobot,30);
robot.DataFormat = 'row';

%% Inverse Kinematics

% Define IK
ik = inverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = jointAnglesHome';
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(robot,robot.homeConfiguration,'Frames','on','PreservePlot',true, 'Collisions', 'on');

hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'r.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'bo','LineWidth',2);

% Solve IK for all waypoints
includeOrientation = false; % Set this to use zero vs. nonzero orientations

numWaypoints = size(waypoints,2);
numJoints = numel(robot.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

for idx = 1:numWaypoints
    if includeOrientation
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(:,idx)');
    else
        tgtPose =  trvec2tform(waypoints(:,idx)');
    end
    [config,info] = ik('body8',tgtPose,ikWeights,ikInitGuess);
    jointWaypoints(:,idx) = config';
end

%% Generate trajectory on joint space

% FAILED TRAJECTORY: Code is for references and has been commented

% trajType = 'quintic';
% switch trajType
%     case 'trap'
%         [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
%             'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
%             'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
%                             
%     case 'cubic'
%         [q,qd,qdd] = cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
%             'VelocityBoundaryCondition',zeros(numJoints,numWaypoints));
%         
%     case 'quintic'
%         [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
%             'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
%             'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
%         
%     case 'bspline'
%         ctrlpoints = jointWaypoints; % Can adapt this as needed
%         [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
%         
%     otherwise
%         error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
% end

% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)

%% Generate Collision Bodies

% Cylindrical Pillars
pillar=collisionCylinder(50, 2000);
pillar.Pose=trvec2tform([1000 0 0]);

pillar2=collisionCylinder(50, 2000);
pillar2.Pose=trvec2tform([-1000 0 0]);

% Cameras for Demonstration
camer1col = collisionBox(100, 100, 100);
camer1col.Pose = [eye(3), [250, 175, 0]'; 0, 0, 0, 1];

camer2col = collisionBox(100, 100, 100);
camer2col.Pose = [eye(3), [-250, 175, 0]'; 0, 0, 0, 1];

% Base Structure
base = collisionBox(1500, 250, 1050);
base.Pose = [eye(3), [0, 0, -475]'; 0, 0, 0, 1];

% Wall Structure
wall = collisionBox(2000, 100, 2000);
wall.Pose = [eye(3), [0, 1950, 0]'; 0, 0, 0, 1];

% Floor for Realism
floor = collisionBox(4000, 3000, 10);
floor.Pose = [eye(3), [0, -500, -1000]'; 0, 0, 0, 1];

%% Define Joint Angles after Computation
% Comment this section when generate joint angle configurations

% q = [];

% Path Planning Joint Angles
% q = csvread('robot_final.csv');

% Trajectory Generation Joint Angles
% q = xlsread('project_qs.xlsx');

%% Trajectory following loop
% Comment this section when defining trajectory generation
% Uncomment this section when plotting the trajectroy with the stored values of q

for idx = 1:size(q, 2)  

    config = q(:,idx)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(robot,config,'body7');
    if plotMode == 1
        eePos = tform2trvec(eeTform);
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
    end

    % Show the robot
    fig = show(robot,config,'Frames','on','PreservePlot',false,  'Collisions', 'on');
    hold on
    [~, pill1] = show(pillar); 
    [~, pill2] = show(pillar2);
    [~, basebod] = show(base);
    [~, cam1bod] = show(camer1col); 
    [~, cam2bod] = show(camer2col);
    [~, pill1] = show(pillar); 
    [~, pill2] = show(pillar2);
    [~, basebod] = show(base);
    [~, cam1bod] = show(camer1col); 
    [~, cam2bod] = show(camer2col);
    [~, wallbod] = show(wall);
    [~, floorbod] = show(floor);
    basebod.FaceColor = [.9 .9 .9];
    cam1bod.FaceColor = [1 0 0];
    cam2bod.FaceColor = [0 0 1];
    pill1.FaceColor = [.9 .9 .9];
    pill2.FaceColor = [.9 .9 .9];
    wallbod.FaceColor = [.9 .9 .9];
    floorbod.FaceColor = [.9 .9 .9];
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    view([180 -180 110])
    xlim([-1500 1500])
    ylim([-1000 2200])
    zlim([-1500 1000])
    set(gcf,'color','w')
    set(gca,'fontname','times')
    zlabel('z    ','rotation',0)
    plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [250 175 0], 'Color', 'red');
    plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [-250, 175, 0], 'Color', 'blue');

    % Create Frames for GIF
    frame = getframe(fig);
    im{idx} = frame2im(frame);
    drawnow   
    
end

%% Camera Calibration Point Plotting
% Comment this section for ease of plotting if camera points are not required
poa1 = camera_robot([52, 953], [346, 953]);
poa1(2) = poa1(2)+175;
poa1(3) = poa1(3)+175;

poa2 = camera_robot([493, 512], [787, 512]);
poa2(2) = poa2(2)+175;
poa2(3) = poa2(3)+175;

poa3 = camera_robot([934, 953], [1228, 953]);
poa3(2) = poa3(2)+175;
poa3(3) = poa3(3)+175;

funbot = interactiveRigidBodyTree(robot, 'MarkerScaleFactor', 1000)

hold on
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'bo','LineWidth',2);
[~, pill1] = show(pillar); 
[~, pill2] = show(pillar2);
[~, basebod] = show(base);
[~, cam1bod] = show(camer1col); 
[~, cam2bod] = show(camer2col);
[~, pill1] = show(pillar); 
[~, pill2] = show(pillar2);
[~, basebod] = show(base);
[~, cam1bod] = show(camer1col); 
[~, cam2bod] = show(camer2col);
[~, wallbod] = show(wall);
[~, floorbod] = show(floor);
basebod.FaceColor = [.9 .9 .9];
cam1bod.FaceColor = [1 0 0];
cam2bod.FaceColor = [0 0 1];
pill1.FaceColor = [.9 .9 .9];
pill2.FaceColor = [.9 .9 .9];
wallbod.FaceColor = [.9 .9 .9];
floorbod.FaceColor = [.9 .9 .9];
plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [250 175 0], 'Color', 'red');
plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [-250, 175, 0], 'Color', 'blue');
show(robot, 'Collisions', 'on')
plot3(poa1(1), poa1(3), poa1(2), 'xb', 'LineWidth', 2)
plot3(poa2(1), poa2(3), poa2(2), 'xb', 'LineWidth', 2)
plot3(poa3(1), poa3(3), poa3(2), 'xb', 'LineWidth', 2)
hold off

%% Trajectory Generation
% Generation of Joint Angles in Cartesian Motion

trajType = 'quintic';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree

if plotMode == 1
    set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
elseif plotMode == 2
    plotTransforms(q',repmat([1 0 0 0],[size(q,2) 1]),'FrameSize',0.05);
end

To visualize the trajectory, run the following line
plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% Trajectory following loop
% Generate Trajectory to define qs 
% Uncomment when performing Trajectory Generation
% Comment when plotting with stored qs

ROBOTCONFIG = zeros(7, numel(trajTimes)-1); % Store q configuration
for idx = 1:numel(trajTimes) 
    % Solve Inverse Kinematics
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik('body8',tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    ROBOTCONFIG(:,idx) = config';
end

% Show the robot
show(robot,config,'Frames','on','PreservePlot',false,'Collisions', 'on');
hold on
[~, pill1] = show(pillar); 
[~, pill2] = show(pillar2);
[~, basebod] = show(base);
[~, cam1bod] = show(camer1col); 
[~, cam2bod] = show(camer2col);
[~, pill1] = show(pillar); 
[~, pill2] = show(pillar2);
[~, basebod] = show(base);
[~, cam1bod] = show(camer1col); 
[~, cam2bod] = show(camer2col);
[~, wallbod] = show(wall);
basebod.FaceColor = [.9 .9 .9];
cam1bod.FaceColor = [1 0 0];
cam2bod.FaceColor = [0 0 1];
pill1.FaceColor = [.9 .9 .9];
pill2.FaceColor = [.9 .9 .9];
wallbod.FaceColor = [.9 .9 .9];
plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [250 175 0], 'Color', 'red');
plotCamera('Size', 100, 'Orientation', rotx(90*pi/180), 'Location', [-250, 175, 0], 'Color', 'blue');
title(['Trajectory at t = ' num2str(trajTimes(idx))])
view([180 -180 110])
xlim([-2500 2500])
ylim([-2500 2500])
zlim([-2500 2500])
drawnow    

%% Save GIF
% Comment this section when performing trajectory generation
% Uncomment when plotting trajectory generation with stored qs
filename = 'robot_finaldrag.gif'; % Specify the output file name
for idx = 1:size(q,2)
    [A,map] = rgb2ind(im{idx},256);
    if idx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',.1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',.1);
    end
end