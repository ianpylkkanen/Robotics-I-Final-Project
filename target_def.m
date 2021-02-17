% planar target points in the reference frame (total Nx*Ny points)
z0=0; % z-coordinate of the points (planar, so 0)
nx1=-.3;kx=.05;nx2=.3; % arrays in x
ny1=-.3;ky=.05;ny2=.3; % arrays in y
% nx1=-.1;kx=.1;nx2=.1; % arrays in x
% ny1=-.1;ky=.1;ny2=.1; % arrays in y
px=(nx1:kx:nx2);Nx=length(px);
py=(ny1:ky:ny2);Ny=length(py);
Pxy=kron([zeros(1,Nx);ones(1,Nx)],py)+...
  kron(px,[ones(1,Ny);zeros(1,Ny)]);
N = Nx*Ny;
P0=[Pxy;z0*ones(1,Nx*Ny)]; % pack into a 2x(Nx*Ny) vector
