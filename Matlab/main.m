clc
clear
close all;

n = 10;
m = 10;
% Attributes of 3 types of robots
R1 = [1 1 1]';
R2 = [1 1 1]';
R3 = [1 1 1]';

% Requirements of 3 types of tasks
T1 = [1 1 1]';
T2 = [1 1 1]';
T3 = [1 1 1]';

Rx = [R1 R2 R3];
Tx = [T1 T2 T3];

% Each column represents attributes of robot i
for ii = 1:n
    theta(:,ii) = Rx(:,floor(3*rand(1)+1));
end

% Each column represents requirements of task j
for ii = 1:m
    psi(:,ii) = Tx(:,floor(3*rand(1)+1));
end

% Spawn locations of robots and tasks


rob_x0 = 10*rand(1,n);
rob_y0 = 10*rand(1,n);
task_x0 = 10*rand(1,m);
task_y0 = 10*rand(1,m);

% rob_x0 = [4,5,6];
% rob_y0 = [1,1,1];
% task_x0 = [4,5,6];
% task_y0 = [4,4,4];
%
init_x = [rob_x0,rob_y0];
init_task = [task_x0,task_y0];
x0 = [init_x init_task];


[argvalue_bot, argmax_bot] = max(theta(:,:));
[argvalue_task, argmax_task] = max(psi(:,:));

featcolors = ['b','g','r'];

tspan = 0:0.1:100;
range = 20;

[t,XY] = ode45(@(t,x) odefcn(t,x,n,m,range,theta,psi),tspan,x0);

xDEMO = XY(:,1:size(XY,2)/2);
yDEMO = XY(:,size(XY,2)/2+1:size(XY,2));

hold on
PLOT_ENVIRONMENT(task_x0,task_y0,'x',featcolors(argmax_task))
PLOT_ENVIRONMENT(xDEMO,yDEMO,'.',featcolors(argmax_bot))
title('Title')
hold off


function Eq = odefcn(t, X, n, m, range, theta, psi)
%X: 1xn vector of robot positions at previous time step with entries
%1:n being x coordinates and entries n+1:2*n being y coordinates
%n: number of robots
%m: number of tasks
%tasks: m x 2 matrix with x coordinates in first column and y
%coordinates in second column



%initialize variables
Eq = zeros(2*n, 1);
xR = X(1:n); %robots' x positions
yR = X(n+1:2*n); %robots' y positions
xT = X(n+m+1:end);%tasks' x positions
yT = X(n+m+1:end);%tasks y positions
RR = zeros(n, n); %inter-robot distances
RT = zeros(m, n); %task-robot distances
AR = zeros(n, n);
AT = zeros(m, n);
U = zeros(m, n);
U2 = zeros(m,n);
%take care of later
velocitySaturation = 0.5;
alpha = 4;
close_enough = .001;

%compute robot distances and form adjacency matrix
for i = 1:n
    for j = 1:n
        RR(i, j) = sqrt((xR(j)-xR(i))^2 + (yR(j)-yR(i))^2);
        if RR(i, j) <= range && i ~= j
            AR(i, j) = 1;
        end
    end
end


%compute task distances
for i = 1:m
    for j = 1:n
        RT(i, j) = sqrt((xR(j)-xT(i))^2 + (yR(j)-yT(i))^2);
        if RT(i, j) <= range
            AT(i, j) = 1;
        end
    end
end
RT;

%compute utilities
for i = 1:m
    for j = 1:n
        U(i, j) = dot(theta(:, j), psi(:, i))/(RT(i, j)^alpha);
    end
end
U;
%adjust utilities
for i = 1:m
    for j = 1:n
        neighbors = nodeNeighbors(AR, j);
        tempU = [];
        for k = neighbors
            tempU = [tempU U(i,k)];
            tempU;
        end
        U(i,j) = round(U(i,j),10);
        U2(i, j) = U(i, j) - max(tempU);
    end
end
U2;
%apply dynamics
for j = 1:n
    [maxU, idxTask] = max(U2(:,j));
    %     if maxU > 0
    Eq(j) = xT(idxTask) - xR(j);
    Eq(n+j) = yT(idxTask) - yR(j);
    velocity = sqrt(Eq(j)^2 + Eq(n+j)^2);
    %saturate velocity
    if velocity > 0.5
        Eq(j) = 0.5*Eq(j)/velocity;
        Eq(n+j) = 0.5*Eq(n+j)/velocity;
    end
    %todo
    %     else
    %         Eq(j) = 0;
    %         Eq(n+j) = 0;
    %     end
end
for ii = 1:m
    for jj = 1:n
        if RT(ii,jj) < close_enough
            Eq(ii) = 10000000000*(10*rand(1));
            Eq(ii+m) = 10000000000*(10*rand(1));
        else
            Eq(ii+2*n) = 0;
            Eq(ii+m+2*n) = 0;
        end
    end

end
end

function [neighbors]  = nodeNeighbors(A, robotIndex)
%function takes as inputs an adjacency matrix and robot index and
%outputs indices of robots that belong to the neighbors set
n = length(A);
%determine neighbors
neighbors = [];
for j = 1:n
    if A(robotIndex, j) == 1
        neighbors = [neighbors j];
    end
end
end

function PLOT_ENVIRONMENT(x,y,FeatShape,FeatColors)

if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end

for i = 1:total_size
    plot(x(i,:),y(i,:),strcat(FeatColors(1),FeatShape),'Linewidth',2)
    pause(0.01);
end

end