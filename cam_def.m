%
% defining a pinhole camera
%

% intrinsic camera parameters

W = 1280; % width in pixels (1280)  uo=W/2 (center of screen)
H = 1024; % height in pixels (1024) vo=H/2 (center of screen)
rhow = 1e-5; % width per pixel (10um)
rhoh = 1e-5; % height per pixel (10um)
rho=[rhow;rhoh];
% f = .015; % focal length (0.015m)
f = .0075;

phih=atan2(H*rhoh/2,f); % vertical viewing angle range
phiw=atan2(W*rhow/2,f); % horizontal viewing angle range

u0=W/2; %center of image plane in pixel coordinate
v0=H/2; %center of image plane in pixel coordinate
uv0=[u0;v0];

cam=pinholecam(f,rho,uv0);
