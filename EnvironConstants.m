function EnvironConstants
global n k alpha beta gamma offset res y0 tspan tstep lw fs ms anim mov frame;
%Mac Schwager, MIT, 2006
%Contains all of the assigned values for global vaiables

%number of agents
n = 10;

%control gain
k = 5;

%gaussian parameters for phi
alpha = [100; 100]; %strength
beta = [.2; .2]; %standard dev
gamma = [0.5,0.5;0.5,0.5]; %center position
offset = [1; 1]; %constant offset

%boundary of the environment (must be convex with points in ccw order for
%Voronoi to work)

%sim constants
tspan = [0 5]; %time span
tstep = .01;  %solver step size
anim = 1; %animation?
mov = 0; %save jpeg frames to make a movie?
frame = 1; %movie frame counter
res = 5; %centroid calulation resolution
lw = 2; %line width for plots
fs = 14; %font size
ms = 10; %marker size

%Initial Conditions
rand('twister',1);
posx0 = rand(n,1);
posy0 = rand(n,1);
y0 = [posx0', posy0'];
