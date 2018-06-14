clc
clear
close all;

% rng(0,'twister');

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
global mu_vec;
global mumap_vec;

global mu;
global sigma;

global mumap;
global sigmap;

indROB = [];
indTSK = [];
indCOUNTVEC = [];
indCOUNT = 1;
mu_vec = [];
mumap_vec = [];


n = 5; %robots
m = 5; %tasks
homo = 1;
ekf_flag = 3;

Q = 0.001*eye(2);
R = 01;

Q_map = .001*eye(2*m);
R_map = 1;


psi = zeros(3,m);

[theta,init_x,init_task] = INIT(n,m,homo);

rob_x0 = init_x(1:size(init_x,2)/2);
rob_y0 = init_x(size(init_x,2)/2+1:size(init_x,2));
task_x0 = init_task(1:size(init_task,2)/2);
task_y0 = init_task(size(init_task,2)/2+1:size(init_task,2));

TASKX = task_x0;
TASKY = task_y0;
tasks = init_task;
tolerance = 0.01;

[argvalue_bot, argmax_bot] = max(theta(:,:));
[argvalue_task, argmax_task] = max(psi(:,:));

argtask_all = argmax_task;

featcolors = ['b','g','r'];
tottime = 10;
stepSize = 0.1;
range = 10;

mu = [rob_x0;rob_y0];
sigma = repmat(1*eye(2),[1,1,n]);

mumap = [task_x0;task_y0];
sigmap = 1*eye(2*m);

M = m;


if ekf_flag == 1 %EKF LOCALIZATION
    [t,XY] = ode45(@(t,x)ekf_odefcn(t,x,n,m,range,theta,Q,R),0:stepSize:tottime,init_x);
elseif ekf_flag == 2 %PF LOCALIZATION
    [t,XY] = ode45(@(t,x)pf_odefcn(t,x,n,m,range,theta,Q,R,M),0:stepSize:tottime,init_x);
elseif ekf_flag == 3 %MULTISLAM
    [t,XY] = ode45(@(t,x)MULTISLAM_odefcn(t,x,n,m,range,theta,Q,R,M,Q_map,R_map),0:stepSize:tottime,init_x);
else
    [t,XY] = ode45(@(t,x)odefcn(t,x,n,m,range,theta),0:stepSize:tottime,init_x);
end

time = toc;

indBOTH = [indROB;indTSK;indCOUNTVEC];

x = XY(:,1:size(XY,2)/2);
y = XY(:,size(XY,2)/2+1:size(XY,2));

t_task = linspace(0,tottime,length(TASKX(:,1)));
% t_task2 = round(t_task,1);
task_x = [t_task',TASKX];
task_y = [t_task',TASKY];

ind = findind(0:stepSize:tottime,t_task);
% [A,ind] = unique(task_x(:,1),'rows','first');
task_x2 = task_x(ind,:);
task_y2 = task_y(ind,:);
task_type = argtask_all(ind,:);

task_x3 = task_x2(:,(2:end));
task_y3 = task_y2(:,(2:end));

if isempty(mumap_vec) == 0 && isempty(mu_vec) == 0
    
    for j = 1:size(mu_vec,2)
        for i = 1:size(mu_vec,1)/2
            xMU(i,j) = mu_vec(2*i-1,j);
            yMU(i,j) = mu_vec(2*i,j);
        end
    end
    
    for j = 1:size(mumap_vec,2)
        for i = 1:size(mumap_vec,1)/2
            xMAP(i,j) = mumap_vec(2*i-1,j);
            yMAP(i,j) = mumap_vec(2*i,j);
        end
    end
    
    t_mu = linspace(0,tottime,length(xMU(:,1)));
    % t_mu2 = round(t_mu,1);
    xMU = [t_mu',xMU];
    yMU = [t_mu',yMU];
    
    ind = findind(0:stepSize:tottime,t_mu);
    % [A,ind] = unique(xMU(:,1),'rows','last');
    
    xMU = xMU(ind,:);
    yMU = yMU(ind,:);
    
    xMU = xMU(:,(2:end));
    yMU = yMU(:,(2:end));
    
    xMAP = xMAP(ind,:);
    yMAP = yMAP(ind,:);
    
end

plottingPrompt = input('Plot animation? (y=1/n=0): ');

fprintf('RunTime: %2.2f secs\n',time)

if plottingPrompt == 1
    figure(1)
    axis([0 10 0 10])
    hold all
    %         M = PLOT_ERROR(x,y,n,'o',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type,xMU,yMU,xMAP,yMAP);
    %     PLOT_ENVIRONMENT(x,y,n,'o',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type);
    %     M = PLOT_ENVIRONMENT_MOV(x,y,n,'o',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type);
    PLOT_PATHS(x,y,n,'.',featcolors,task_x3,task_y3,m,'x',argmax_bot,task_type,homo);
    %     [vx,vy] = voronoi(x(1,:),y(1,:));
    %     VOR = plot(vx,vy,'k');
    hold off
end

task_comp = t_task(indTSK);
task_comp = findind(task_comp,0:stepSize:tottime);
task_comp = t(task_comp);

if isempty(mumap_vec) == 0 && isempty(mu_vec) == 0
    
%     figure
%     subplot(2,1,1)
%     for i = 1:size(x,2)
%         hold on
%         plot(t,abs(x(:,i)-xMU(:,i))/x(:,i)*100)
%     end
%     for i = 1:length(task_comp)
%         plot([task_comp(i),task_comp(i)],[0,50],'k--')
%     end
%     legend('R1','R2','R3','R4','R5','Location','Best')
%     hold off;
%     title('Localization Estimation Error vs Time')
%     xlabel('Time (sec)')
%     ylabel('X-Position Error (%)')
%     subplot(2,1,2)
%     for i = 1:size(y,2)
%         hold on
%         plot(t,abs(y(:,i)-yMU(:,i))/y(:,i)*100)
%     end
%     for i = 1:length(task_comp)
%         plot([task_comp(i),task_comp(i)],[0,50],'k--')
%     end
%     hold off;
%     xlabel('Time (sec)')
%     ylabel('X-Position Error (%)')
%     legend('R1','R2','R3','R4','R5','Location','Best')
    
    
    figure
    for i = 1:size(x,2)
        hold on
        plot(t,vecnorm([abs(x(:,i)-xMU(:,i))/x(:,i)*100;abs(y(:,i)-yMU(:,i))/y(:,i)*100]))
    end
    for i = 1:length(task_comp)
        plot([task_comp(i),task_comp(i)],[0,50],'k--')
    end
    hold off;
    xlabel('Time (sec)')
    ylabel('X-Position Error (%)')
    
end

F2 = figure(2);
axis([0 10 0 10])
hold all
if homo == true
    for i = 1
        for j = 1:n
            ROBERR(i,j) = plot(xMU(i,j),yMU(i,j),'b.','LineWidth',2);
            TSKERR(i,j) = plot(xMAP(i,j),yMAP(i,j),'r.');
            plot(x(i,j),y(i,j),'k*','Linewidth',2);
%             plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
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
            ROBERR(i,j) = plot(xMU(i,j),yMU(i,j),'b.','LineWidth',2);
            TSKERR(i,j) = plot(xMAP(i,j),yMAP(i,j),'r.');
            plot(x(i,j),y(i,j),'k.','Linewidth',1);
%             plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
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
            ROBERR(i,j) = plot(xMU(i,j),yMU(i,j),'b.','LineWidth',2);
            TSKERR(i,j) = plot(xMAP(i,j),yMAP(i,j),'r.');
            plot(x(i,j),y(i,j),'k.','Linewidth',1);
%             plot(task_x3(i,j),task_y3(i,j),'rx','Linewidth',2);
            plot(x(total_size,j),y(total_size,j),'k*','Linewidth',2);
        end
    end
else
    for i = 1:total_size
        for j = 1:n
            plot(x(i,j),y(i,j),strcat(featcolors(argmax_bot(j)),'.'),'Linewidth',1);
            plot(task_x3(i,j),task_y3(i,j),strcat(featcolors(task_type(i,j)),'x'),'Linewidth',2);
            plot(x(total_size,j),y(total_size,j),strcat(featcolors(argmax_bot(j)),'*'),'Linewidth',2);
            ROBERR(i,j) = plot(xMU(i,j),yMU(i,j),'g.','LineWidth',2);
            TSKERR(i,j) = plot(xMAP(i,j),yMAP(i,j),'r.','LineWidth',2);

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

