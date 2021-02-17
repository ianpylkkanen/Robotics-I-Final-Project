%% Camera Position Finding
%% This code takes in world coordinate positions and projects them into the image frame to determine image coordinates for two cameras

cam_def;
cam1 = cam;
cam2 = cam;

poc1 = [-.250, 0, .175]';
poc2 = [.250, 0, .175]';

% Poa1 = [-.5, .5, 1.9]';
Poa1 = [.25, .25, 1.9]';
% Poa1 = [.75, .75, 1.9]';

% Poa2 = [-.5, .5, 1.9]';
Poa2 = [.25, .25, 1.9]';
% Poa2 = [.75, .75, 1.9]';

Kest = camcalib();

pca1 = Poa1 - poc1;
pca2 = Poa2 - poc2;

uvz1 = cam1.K*pca1/pca1(3);
uvz2 = cam2.K*pca2/pca2(3);

uvz1e = Kest*pca1/pca1(3)
uvz2e = Kest*pca2/pca2(3)


uv1 = uvz1
uv2 = uvz2

error1 = abs(norm(uvz1e) - norm(uvz1))
error2 = abs(norm(uvz1e) - norm(uvz1))

