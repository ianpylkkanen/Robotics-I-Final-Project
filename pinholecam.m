%
% pinholecam.m
%
% generates a pinhole camera object
%
% usage: cam=pinholecam(f,rhox,rhoy,u0,v0)
% 
% input: 
% f: focal length
% rhox: width/pixel
% rhoy: height/pixel
% u0,v0: pixel domain location of optical center
% 
% output:
% cam: camera object with attributes f, rhox, rhoy, u0, v0, and
% K=intrinsic camera matrix
% 

function cam=pinholecam(f,rho,uv0)

cam.f=f;
cam.rho=rho;
cam.uv0=uv0;
cam.K=[f/rho(1) 0 uv0(1);0 f/rho(2) uv0(2);0 0 1];

end



