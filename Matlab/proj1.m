clc
clear
close all;

rng(2,'twister');

tic
 
global tasks
global tolerance
global psi
global argmax_task
global argmax_bot
global TASKX
global TASKY
global argtask_all
global indROB;
global indTSK;
global indCOUNTVEC;
global indCOUNT;

indROB = [];
indTSK = [];
indCOUNTVEC = [];
indCOUNT = 1;

n = 3;
m = 3;
homo = 0;

psi = zeros(3,m);

[theta,init_x,init_task] = INIT(n,m,homo);

rob_x0 = init_x(1:size(init_x,2)/2);
rob_y0 = init_x(size(init_x,2)/2+1:size(init_x,2));
task_x0 = init_task(1:size(init_task,2)/2);
task_y0 = init_task(size(init_task,2)/2+1:size(init_task,2));

TASKX = task_x0;
TASKY = task_y0;
tasks = init_task;
tolerance = 0.001;

[argvalue_bot, argmax_bot] = max(theta(:,:));
[argvalue_task, argmax_task] = max(psi(:,:));

argtask_all = argmax_task;

featcolors = ['b','g','r'];
tottime = 50;
tspan = 0:0.1:tottime;
range = 100;

[t,XY] = ode45(@(t,x) odefcn(t,x,n,m,range,theta),tspan,init_x);

indBOTH = [indROB;indTSK;indCOUNTVEC];

x = XY(:,1:size(XY,2)/2);
y = XY(:,size(XY,2)/2+1:size(XY,2));

t_task = linspace(0,tottime,length(TASKX(:,1)));
t_task2 = round(t_task,1);
task_x = [t_task2',TASKX];
task_x_init = [t_task2',TASKX];
task_y_init = [t_task2',TASKY];
task_y = [t_task2',TASKY];

[A,ind] = unique(task_x(:,1),'rows','first');
task_x2 = task_x(ind,:);
task_y2 = task_y(ind,:);
task_type = argtask_all(ind,:);

task_x3 = task_x2(:,(2:end));
task_y3 = task_y2(:,(2:end));
%
% task_count = -m;
%
% for i = 1:length(task_x3(1,:))
%     task_count = task_count + length(unique(task_x3(:,i)));
% end
%
% indBOTH = indBOTH';
% HIT = [];
%
% for i = 1:length(indBOTH(:,1))
%     for j = 1:length(ind)
%         if ind(j) == indBOTH(i,3)
%             HIT = [HIT;indBOTH(i,(1:2))];
%         end
%     end
% end
%
% TOTALHIT = [HIT(1,:)];
%
% for i = 1:length(HIT(:,1))-1
%     if sum(HIT(i,1)-HIT(i+1,1))~=0 || sum(HIT(i,2)-HIT(i+1,2))~=0
%         TOTALHIT = [TOTALHIT;HIT(i+1,:)];
%     end
% end
%
% HETERO_COUNT = 0;
%
% for i = 1:length(TOTALHIT(:,1))
%     if TOTALHIT(i,1) == TOTALHIT(i,2)
%         HETERO_COUNT = HETERO_COUNT + 1;
%     end
% end

% hdt = datacursormode;
% set(hdt,'DisplayStyle','window');
% set(hdt,'UpdateFcn',{@labeldtips,x,y,init_task,theta,psi})

figure(1)
axis([0 10 0 10])
hold all
%PLOT_ENVIRONMENT(x,y,n,'o',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type);
% M = PLOT_ENVIRONMENT_MOV(x,y,n,'o',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type);
PLOT_PATHS(x,y,n,'.',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type,homo);
[vx,vy] = voronoi(x(1,:),y(1,:));
VOR = plot(vx,vy,'k');
hold off

F2 = figure(2);
axis([0 10 0 10])
hold all
if homo == true
    for i = 1
        for j = 1:n
            plot(x(i,j),y(i,j),'k*','Linewidth',2);
            plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
        end
    end
else
    for i = 1
        for j = 1:n
            plot(x(i,j),y(i,j),strcat(featcolors(argmax_bot(j)),'*'),'Linewidth',2);
            plot(task_x3(i,j),task_y3(i,j),strcat(featcolors(task_type(i,j)),'x'),'Linewidth',2);
        end
    end
end
[vx,vy] = voronoi(x(1,:),y(1,:));
VOR = plot(vx,vy,'k');
hold off

F3 = figure(3);
axis([0 10 0 10])
hold all
if homo == true
    for i = 1:floor(length(x)/3)
        for j = 1:n
            plot(x(i,j),y(i,j),'k.','Linewidth',1);
            plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
            plot(x(floor(length(x)/3),j),y(floor(length(x)/3),j),'k*','Linewidth',2);
        end
    end
else
    for i = 1:floor(length(x)/3)
        for j = 1:n
            plot(x(i,j),y(i,j),strcat(featcolors(argmax_bot(j)),'.'),'Linewidth',1);
            plot(task_x3(i,j),task_y3(i,j),strcat(featcolors(task_type(i,j)),'x'),'Linewidth',2);
            plot(x(floor(length(x)/3),j),y(floor(length(x)/3),j),strcat(featcolors(argmax_bot(j)),'*'),'Linewidth',2);
        end
    end
end
[vx,vy] = voronoi(x(floor(length(x)/3),:),y(floor(length(y)/3),:));
VOR = plot(vx,vy,'k');
hold off
% 
F4 = figure(4);
axis([0 10 0 10])
hold all
if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end
if homo == true
    for i = 1:total_size
        for j = 1:n
            plot(x(i,j),y(i,j),'k.','Linewidth',1);
            plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
            plot(x(total_size,j),y(total_size,j),'k*','Linewidth',2);
        end
    end
else
    for i = 1:total_size
        for j = 1:n
            plot(x(i,j),y(i,j),strcat(featcolors(argmax_bot(j)),'.'),'Linewidth',1);
            plot(task_x3(i,j),task_y3(i,j),strcat(featcolors(task_type(i,j)),'x'),'Linewidth',2);
            plot(x(total_size,j),y(total_size,j),strcat(featcolors(argmax_bot(j)),'*'),'Linewidth',2);
        end
    end
end
[vx,vy] = voronoi(x(end,:),y(end,:));
VOR = plot(vx,vy,'k');
hold off

figure(5)
h(1)=subplot(1,3,1);
axis([0 10 0 10])
h(2)=subplot(1,3,2);
axis([0 10 0 10])
h(3)=subplot(1,3,3);
axis([0 10 0 10])
copyobj(allchild(get(F2,'CurrentAxes')),h(1));
copyobj(allchild(get(F3,'CurrentAxes')),h(2));
copyobj(allchild(get(F4,'CurrentAxes')),h(3));

time = toc;

fprintf('RunTime: %2.2f secs\n',time)
% fprintf('Tasks: %2.2f\n',task_count)
% fprintf('Timestep/Task: %2.2f\n',length(x(:,1))/task_count)
% fprintf('Hetero/Total: %2.2f\n',HETERO_COUNT/task_count)


function [theta,x0,t0] = INIT(n,m,homo)

global Tx
global psi

val = 50;
fact = 2;

% Attributes of 3 types of robots
R1 = [val floor(fact*rand+1) floor(fact*rand+1)]';
R2 = [floor(fact*rand+1) val floor(fact*rand+1)]';
R3 = [floor(fact*rand+1) floor(fact*rand+1) val]';

% Requirements of 3 types of tasks
T1 = [val floor(fact*rand+1) floor(fact*rand+1)]';
T2 = [floor(fact*rand+1) val floor(fact*rand+1)]';
T3 = [floor(fact*rand+1) floor(fact*rand+1) val]';

if homo == true
    Rx = [R1 R1 R1];
    Tx = [T1 T1 T1];
elseif homo == false
    Rx = [R1 R2 R3];
    Tx = [T1 T2 T3];
end

% Each column represents attributes of robot i
for ii = 1:n
    theta(:,ii) = Rx(:,floor(3*rand(1)+1));
end

% Each column represents requirements of task j
for ii = 1:m
    psi(:,ii) = Tx(:,floor(3*rand(1)+1));
end

% Spawn locations of robots and tasks
x0 = rand(1,2*n)*10;
t0 = rand(1,2*m)*10;

end

function Eq = odefcn(t, X, n, m, range, theta)
%X: 1xn vector of robot positions at previous time step with entries
%1:n being x coordinates and entries n+1:2*n being y coordinates
%n: number of robots
%m: number of tasks
%tasks: m x 2 matrix with x coordinates in first column and y
%coordinates in second column

global tasks
global psi

%initialize variables
Eq = zeros(2*n, 1);
xR = X(1:n); %robots' x positions
yR = X(n+1:2*n); %robots' y positions
xT = tasks(1:size(tasks,2)/2);%tasks' x positions
yT = tasks(size(tasks,2)/2+1:size(tasks,2));%tasks y positions
RR = zeros(n, n); %inter-robot distances
RT = zeros(m, n); %task-robot distances
AR = zeros(n, n);
AT = zeros(m, n);
U = zeros(m, n);
U2 = zeros(m,n);

%take care of later
velocitySaturation = 0.5;
alpha = 3;
beta = 1;

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

%compute utilities
for i = 1:m
    for j = 1:n
        U(i, j) = (dot(theta(:, j), psi(:, i))^beta)/(RT(i, j)^alpha);
    end
end

%adjust utilities
for i = 1:m
    for j = 1:n
        neighbors = nodeNeighbors(AR, j);
        tempU = [];
        for k = neighbors
            tempU = [tempU U(i,k)];
        end
        U2(i, j) = U(i, j) - max(tempU);
    end
end

%apply dynamics
for j = 1:n
    [maxU, idxTask] = max(U2(:,j));
    if maxU > 0
        Eq(j) = (xT(idxTask) - xR(j));
        Eq(n+j) = yT(idxTask) - yR(j);
        velocity = sqrt((Eq(j))^2 + Eq(n+j)^2);
        %saturate velocityvelocitySaturation
        if velocity > velocitySaturation
            Eq(j) = velocitySaturation*Eq(j)/velocity;
            Eq(n+j) = velocitySaturation*Eq(n+j)/velocity;
        end
        %todo
    else
        [V, Ni] = Voronoi([xR yR], [xR(j); yR(j)]); %Decentralized Voronoi
        [Mv, Lv] = CentroidNumerical(V); %Calculate centrois of Voronoi region using numerical method
        Eq(j) = (Lv(1)/Mv - xR(j)); %Implement control law (x-component)
        Eq(j+n) = (Lv(2)/Mv - yR(j)); %Implement control law (y-component)
    end
end

updateTasks(X, m, n)

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

function M = PLOT_ENVIRONMENT_MOV(x,y,num,FeatShape,FeatColors,xt,yt,numt,FeatShapet,bot_type,task_type)
if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end

for i = 1:total_size
    for j = 1:num
        ROB(i,j) = plot(x(i,j),y(i,j),strcat(FeatColors(bot_type(j)),FeatShape),'Linewidth',2);
    end
    for j = 1:numt
        TSK(i,j) = plot(xt(i,j),yt(i,j),strcat(FeatColors(task_type(i,j)),FeatShapet),'Linewidth',2);
    end
    [vx,vy] = voronoi(x(i,:),y(i,:));
    VOR = plot(vx,vy,'k');
    M(i) = getframe;
    set(VOR,'Visible','off')
    set(ROB(i,:),'Visible','off')
    set(TSK(i,:),'Visible','off')
end

end

function PLOT_ENVIRONMENT(x,y,num,FeatShape,FeatColors,xt,yt,numt,FeatShapet,bot_type,task_type)

if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end

for i = 1:total_size
    for j = 1:num
        ROB(i,j) = plot(x(i,j),y(i,j),strcat(FeatColors(bot_type(j)),FeatShape),'Linewidth',2);
        TSK(i,j) = plot(xt(i,j),yt(i,j),strcat(FeatColors(task_type(i,j)),FeatShapet),'Linewidth',2);
    end
    [vx,vy] = voronoi(x(i,:),y(i,:));
    VOR = plot(vx,vy,'k');
    pause(0.01);
    set(VOR,'Visible','off')
    set(ROB(i,:),'Visible','off')
    set(TSK(i,:),'Visible','off')
end

end

function PLOT_PATHS(x,y,num,FeatShape,FeatColors,xt,yt,numt,FeatShapet,bot_type,task_type,homo)

if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end
if homo == true
    for i = 1:total_size
        for j = 1:num
            plot(x(i,j),y(i,j),'k.','Linewidth',2);
            plot(xt(i,j),yt(i,j),'rx','Linewidth',2);
        end
    end
else
    for i = 1:total_size
        for j = 1:num
            plot(x(i,j),y(i,j),strcat(FeatColors(bot_type(j)),'.'),'Linewidth',2);
            plot(xt(i,j),yt(i,j),strcat(FeatColors(task_type(i,j)),'x'),'Linewidth',2);
        end
    end
end
end

function output_txt = labeldtips(obj,event_obj,X,Y,task,theta,psi)

pos = get(event_obj,'Position');
x = pos(1); y = pos(2);
output_txt = {['Position: (',num2str(pos(1),3),', ',num2str(pos(2),3),')']};

if event_obj.Target.Marker == 'x'
    output_txt{end+1} = 'Object: Task';
else
    output_txt{end+1} = 'Object: Robot';
end

if event_obj.Target.Color == [1 0 0]
    output_txt{end+1} = 'Type: Red';
elseif event_obj.Target.Color == [0 1 0]
    output_txt{end+1} = 'Type: Green';
elseif event_obj.Target.Color == [0 0 1]
    output_txt{end+1} = 'Type: Blue';
else
    output_txt{end+1} = 'Type: Else';
end

if event_obj.Target.Marker == 'x'
    ind = find(task == x);
    output_txt{end+1} = ['Requirements: (',...
        num2str(psi(1,ind)),', ',...
        num2str(psi(2,ind)),', ',...
        num2str(psi(3,ind)),')'];
else
    indx = find(X == x);
    ind = [mod(indx,size(X,1)) floor(indx/size(X,1))+1];
    output_txt{end+1} = ['Attributes: (',...
        num2str(theta(1,ind(2))),', ',...
        num2str(theta(2,ind(2))),', ',...
        num2str(theta(3,ind(2))),')'];
end

end

function updateTasks(robotPositions, m,n)
%Function updates available tasks on the map. A task is removed once a
%robot reaches a distance of less than or equal to tolerance of it. If
%the current number of available tasks is less than the maximum number of
%tasks, m, the function adds tasks until there are m tasks.

global tasks;
global tolerance;
global psi;
global Tx;
global TASKX;
global TASKY;
global argmax_task;
global argmax_bot;
global argtask_all;
global indROB;
global indTSK;
global indCOUNTVEC;
global indCOUNT;

%initialize variables
xTasks = tasks(1:size(tasks,2)/2);
yTasks = tasks(size(tasks,2)/2+1:size(tasks,2));
xRobots = robotPositions(1:n);
yRobots = robotPositions(n+1:2*n);

%check to see if any tasks are accomplished and remove them
numExistingTasks = length(xTasks);
currentTask = 1;
numRobots = length(xRobots);
while currentTask < numExistingTasks
    skip = false;
    for j = 1:numRobots
        distance = sqrt((xRobots(j) - xTasks(currentTask))^2 +...
            (yRobots(j) - yTasks(currentTask))^2);
        if distance <= tolerance
            indROB = [indROB,argmax_bot(j)];
            indTSK = [indTSK,argmax_task(currentTask)];
            indCOUNTVEC = [indCOUNTVEC,indCOUNT];
            xTasks(currentTask) = [];
            yTasks(currentTask) = [];
            psi(:,currentTask) = [];
            argmax_task(currentTask) = [];
            skip = true;
            break
        end
    end
    if ~skip
        currentTask = currentTask + 1;
    end
    numExistingTasks = length(xTasks);
end

%add new tasks
numNewTasks = m - numExistingTasks;

if numNewTasks > 0
    newPsi = zeros(length(Tx(:,1)),numNewTasks);
    for i = 1:numNewTasks
        newPsi(:,i) = Tx(:,floor(3*rand(1)+1));
    end
    
    newTasks = rand(1,2*numNewTasks)*10;
    
    newXTasks = newTasks(1:size(newTasks,2)/2);
    newYTasks = newTasks(size(newTasks,2)/2+1:size(newTasks,2));
    
    %assign output
    tasks = [xTasks newXTasks yTasks newYTasks];
    psi = [psi newPsi];
    [~, argmax_task] = max(psi(:,:));
    
end

argtask_all = [argtask_all;argmax_task];
TASKX = [TASKX;tasks(1:size(tasks,2)/2)];
TASKY = [TASKY;tasks(size(tasks,2)/2+1:size(tasks,2));];
indCOUNT = indCOUNT+1;

end

function [V0, NI] = Voronoi(P0, p0)
%Computes the voronoi region of the robot at p given the positions
%of all the other robots P

EnvironConstants;

%Constants
Limit = 10;
tol = 1e-6;
n = length(P0(:,1))-1;
p0 = p0';

%Find Distance and sort
D = P0 - ones(n+1,1)*p0;
D = sqrt(D(:,1).^2 + D(:,2).^2);
[D, I] = sort(D);

%Take p0 out of P0 and put in order of distance
P = P0(I(2:n+1),:);
N = I(2:n+1); %indices of neighbors in P0

%Incremental Voronoi Comp

%Start with whole region Q
V0 = [0 0; 0 10; 10 10; 10 0]; %unit square
l = length(V0(:,1));
V0 = [V0; V0(1,:)];
Lines = zeros(l,3);
for i = 1:l
    Lines(i,:) = GetLine(V0(i,:)', V0(i+1,:)');
end
V0 = V0(1:l,:);

i = 1;
NI = zeros(l,1);
d = Limit;
LSize = length(Lines(:,1));
VSize = length(V0(:,1));

%while there are still points, and a) the region is unbounded or b) the
%farthest vertex is more than half the distance to the next point
while (i<=n) && (((2*d)>D(i)) || (LSize>VSize))
    NI = [NI; N(i)]; %add index to neighbor list
    pi = P(i,:); %ith point
    Line = VLine(pi', p0')'; %ith line
    Lines = [Lines; Line]; %Add Line to Lines
    %loop to find intersection points that are inside the half planes of
    %all Lines
    for j = 1:LSize
        Inpt = LineIntersect(Lines(j,:)', Line')'; %find intersection point j
        V0 = [V0; Inpt]; %Add Inpt to V0
        VSize = length(V0(:,1));
        %Test if any points are inside all half planes
        Test = Lines*[V0, ones(VSize,1)]'<=tol;
        Test = cast(Test, 'double');
        Test = prod(Test,1); %logical AND over the columns
        %Filter out all points not inside all half-planes
        [Test I] = sort(Test, 'descend');
        VSize = sum(Test);
        I = I(1:VSize);
        V0 = V0(I,:);
    end
    %Get rid of lines not containing any points in V0
    Test = abs(Lines*[V0, ones(VSize,1)]')>=tol'; %zero indicates a V0 point on a Line
    Test = cast(Test, 'double')';
    Test = prod(Test,1); %logical AND over the columns
    Test = 1-Test; %logical inverse
    %Filter
    [Test I] = sort(Test, 'descend');
    LSize = sum(Test);
    I = I(1:LSize);
    Lines = Lines(I,:);
    NI = NI(I); %filter out indices of non-Voronoi Neighbors
    %max distance point in V0
    d = max(sqrt((V0(:,1)-p0(1)).^2 + (V0(:,2)-p0(2)).^2));
    %increment
    i = i+1;
end
%put V points in order
angle = atan2((V0(:,2)-p0(2)),(V0(:,1)-p0(1)));
[angle I] = sort(angle);
V0 = V0(I,:);

%Voronoi neighbor indices, put in order
%get rid of zeros
i = 1;
while ~isempty(NI) && NI(i) == 0
    NI = NI(2:length(NI));
end
Nangle = atan2((P0(NI,2)-p0(2)),(P0(NI,1)-p0(1)));
[Nangle I] = sort(Nangle);
NI = NI(I);

end