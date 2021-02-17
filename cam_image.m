%
% cam_image: 
%
% generating camera images of a set of 3D points on the image plane for a
% specified pinhole camera
%
% input:
% cam: pinhole camera object
% Toc: camera location in reference frame
% P0: a set of 3D points in reference frame 3xN matrix
%
% output: 
% uv: image coordinate of P0
% uvw: homogenous coordinate of P0
% P1: target points in the camera frame
%

function [uv,uvw,P1]=cam_image(cam,Toc,P0)

K=cam.K; % intrinsic camera matrix
Tco=inv(Toc);
C=K*Tco(1:3,:); %full camera matrix

% points in the camera frame
P1=Tco(1:3,1:3)*P0;

% (u',v',w') 
uvw=C*[P0;ones(1,size(P0,2))];

% image plane coordinate in pixels
u=(uvw(1,:)./uvw(3,:));
v=(uvw(2,:)./uvw(3,:));
uv=[u;v];

% only keep points within the image plane
uv=uv(:,uv(1,:)<2*cam.uv0(1));
uv=uv(:,uv(2,:)<2*cam.uv0(2));
uv=uv(:,uv(1,:)>0);
uv=uv(:,uv(2,:)>0);

end




