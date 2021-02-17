% function [col_pts, robot] = collisioncheck_7dof()

% This function checks the collision status for all possible joint angles within limits, for the first 3 degrees of freedom
% The output is the set of angles for which collisions occur

%% Robot Definition
close all

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

%% Collision Bodies Definition
pillar=collisionCylinder(50, 2000);
pillar.Pose=trvec2tform([1000 0 0]);

pillar2=collisionCylinder(50, 2000);
pillar2.Pose=trvec2tform([-1000 0 0]);

camer1col = collisionBox(100, 100, 100);
camer1col.Pose = [eye(3), [500, 175, 0]'; 0, 0, 0, 1];

camer2col = collisionBox(100, 100, 100);
camer2col.Pose = [eye(3), [-500, 175, 0]'; 0, 0, 0, 1];

base = collisionBox(1500, 250, 1200);
base.Pose = [eye(3), [0, 0, -400]'; 0, 0, 0, 1];
%% Collision Check

ngrid=20;qlim=3*pi/2;
q1min=-qlim;q1max=qlim;dq1=(q1max-q1min)/ngrid;
q2min=-qlim;q2max=qlim;dq2=(q2max-q2min)/ngrid;
q3min=-qlim;q3max=qlim;dq3=(q3max-q3min)/ngrid;

q1=(q1min:dq1:q1max);
q2=(q2min:dq2:q2max);
q3=(q3min:dq3:q3max);

nq1=numel(q1);
nq2=numel(q2);
nq3=numel(q3);

col_pts = zeros(1,3);
h = 1;

for i=1:nq1
    for j=1:nq2
        for k=1:nq3
%            [isInt,dist,wp]=collisionPlot(rob,robdef,[q1(i);q2(j)],colLink,obs,fignum);
           [isInt,dist,wp]=collisionCheck(myrobot,[q1(i);q2(j);q3(k);0;0;0;0],colLink,pillar);
           [isInt2,dist2,wp2]=collisionCheck(myrobot,[q1(i);q2(j);q3(k);0;0;0;0],colLink,pillar2);
           [isInt3,dist3,wp3]=collisionCheck(myrobot,[q1(i);q2(j);q3(k);0;0;0;0],colLink,base);
           isInt  = cell2mat(isInt);
           isInt2 = cell2mat(isInt2);
           isInt3 = cell2mat(isInt3);
           isInt_total = isInt | isInt2 | isInt2;
           p = 0;
           for g = 1:length(isInt_total)
              if  isInt_total(g) == 1
                  p = 1;
              end
           end
           if p == 1
                col_pts(h,:) = [q1(i), q2(j), q3(k)];
                h = h+1
           end
        end
    end
end

%%

angles = csvread('robot_backwall.csv');

figure(1)
plot3(col_pts(:,1), col_pts(:,2), col_pts(:,3), 'xr', 'LineWidth', 2)
hold all
% plot3(angles(1,:), angles(2,:), angles(3,:), 'bo-', 'LineWidth', 2)
grid on
xlabel('q1')
ylabel('q2')
zlabel('q3')


figure(2)
show(robot,[ [col_pts(1,:)]' ;0; 0; 0; 0 ] , 'Collisions', 'on')
hold on
[~, pill1] = show(pillar); 
[~, pill2] = show(pillar2);
[~, basebod] = show(base);
basebod.FaceColor = [.9 .9 .9];
pill1.FaceColor = [.9 .9 .9];
pill2.FaceColor = [.9 .9 .9];


% end
