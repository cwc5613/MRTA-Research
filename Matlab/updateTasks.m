function updateTasks(robotPositions,m,n)
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
global argtask_all;
global indCOUNT;
global indTSK;

%initialize variables
xTasks = tasks(1:size(tasks,2)/2);
yTasks = tasks(size(tasks,2)/2+1:size(tasks,2));
xRobots = robotPositions(1:n);
yRobots = robotPositions(n+1:2*n);

%check to see if any tasks are accomplished and remove them
numExistingTasks = length(xTasks);
numRobots = length(xRobots);
for i = 1:numExistingTasks
    for j = 1:numRobots
        distance = sqrt((xRobots(j) - xTasks(i))^2 +...
            (yRobots(j) - yTasks(i))^2);
        if distance <= tolerance
            rng('shuffle','twister')
            xTasks(i) = rand(1,1)*10;
            yTasks(i) = rand(1,1)*10;
            psi(:,i) = Tx(:,floor(3*rand(1)+1));
            indTSK = [indTSK, indCOUNT];
        end
    end
end

[~,IA,~] = unique(xTasks,'stable');

for i = 1:length(xTasks)
    if ismember(i,IA)
    else 
        xTasks(i) = rand(1,1)*10;
    end
end

[~,IA,~] = unique(yTasks,'stable');

for i = 1:length(yTasks)
    if ismember(i,IA)
    else 
        yTasks(i) = rand(1,1)*10;
    end
end

%assign output
tasks = [xTasks yTasks];
[~, argmax_task] = max(psi(:,:));

argtask_all = [argtask_all;argmax_task];
TASKX = [TASKX;tasks(1:size(tasks,2)/2)];
TASKY = [TASKY;tasks(size(tasks,2)/2+1:size(tasks,2));];
indCOUNT = indCOUNT+1;

end