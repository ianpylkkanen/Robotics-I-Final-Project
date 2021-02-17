function [poa, poaerr] = camera_robot(uv1, uv2)
% [poa, poaerr] = camera_robot(uv1, uv2)
% This function solves the pinhole camera solution for target position in world coordinates
% Inputs:
% uv1, uv2: Image coordinates of points of interest
% 
% Outputs: 
% poa: world coordinates
% poaerr: world position from estimated camera calibration

ex = [1, 0, 0]';
ey = [0, 1, 0]';
ez = [0, 0, 1]';

u1 = uv1(1);
v1 = uv1(2);
u2 = uv2(1);
v2 = uv2(2);


cam_def;
cam1 = cam;
cam2 = cam;

Kest = camcalib;

% Rc1 = rotz(10*pi/180);
% Rc2 = rotz(10*-pi/180);
Rc1 = eye(3);
Rc2 = eye(3);
% Rc1 = rotx(90*pi/180)*rotz(180*pi/180);
% Rc2 = rotx(90*pi/180)*rotz(180*pi/180);

pc1 = [-.25, .175, 0]';
pc2 = [.25, .175, 0]';

%Position on Image Plane
% u1 = 1000;
% v1 = 500;
% u2 = 1152;
% v2 = 900;

au1 = ex'*cam1.K*Rc1;
bu1 = ex'*cam1.K*pc1;
av1 = ey'*cam1.K*Rc1;
bv1 = ey'*cam1.K*pc1;
aw1 = ez'*cam1.K*Rc1;
bw1 = ez'*cam1.K*pc1;

au2 = ex'*cam2.K*Rc2;
bu2 = ex'*cam2.K*pc2;
av2 = ey'*cam2.K*Rc2;
bv2 = ey'*cam2.K*pc2;
aw2 = ez'*cam2.K*Rc2;
bw2 = ez'*cam2.K*pc2;

A = [u1*aw1 - au1;
     v1*aw1 - av1;
     u2*aw2 - au2;
     v2*aw2 - av2];
 
b = [bu1 - bw1;
     bv1 - bw1;
     bu2 - bw2;
     bv2 - bw2];

poa = (A'*A)^-1 * A'*b;
poa = poa * 1000;

%%

au1e = ex'*Kest*Rc1;
bu1e = ex'*Kest*pc1;
av1e = ey'*Kest*Rc1;
bv1e = ey'*Kest*pc1;
aw1e = ez'*Kest*Rc1;
bw1e = ez'*Kest*pc1;

au2e = ex'*Kest*Rc2;
bu2e = ex'*Kest*pc2;
av2e = ey'*Kest*Rc2;
bv2e = ey'*Kest*pc2;
aw2e = ez'*Kest*Rc2;
bw2e = ez'*Kest*pc2;

Ae = [u1*aw1e - au1e;
     v1*aw1e - av1e;
     u2*aw2e - au2e;
     v2*aw2e - av2e];
 
be = [bu1e - bw1e;
     bv1e - bw1e;
     bu2e - bw2e;
     bv2e - bw2e];

poaerr = (Ae'*Ae)^-1 * Ae'*be;
poaerr = poaerr * 1000;

% plot3(poa(1), poa(2), poa(3), 'x', 'LineWidth', 2)
% grid
% 
% figure(2)
% hold on
% axis([0 1280 0 1024])
% plot(u1, v1, 'rx', 'LineWidth', 2)
% plot(u2, v2, 'bx', 'LineWidth', 2)
% grid


end
