clear
clc
close all

load DATA_HET2.mat
DATA_HET = DATA;
load DATA_BIN2.mat
DATA_BIN = DATA;
load DATA_HOM.mat
DATA_HOM = DATA;


% figure
% hold on
% plot((DATA_HET(:,5)),DATA_HET(:,2),'ko')
% plot((DATA_BIN(:,5)),DATA_BIN(:,2),'ro')
% plot((DATA_HOM(:,5)),DATA_HOM(:,2),'bo')
% title('Task Duration vs Robot/Task Count')
% xlabel('Robot/Task Count')
% ylabel('Task Duration (timesteps)')
% hold off

figure
hold on

curve1 = fit((DATA_HET(:,5)),DATA_HET(:,4),'poly4');
curve2 = fit((DATA_BIN(:,5)),DATA_BIN(:,4),'poly4');
curve3 = fit((DATA_HOM(:,5)),DATA_HOM(:,4),'poly4');

plot((DATA_BIN(:,5)),DATA_BIN(:,4),'Color',[140, 21, 21]/255,'LineWidth',2)
plot((DATA_HET(:,5)),DATA_HET(:,4),'Color',[20, 20, 20]/255,'LineWidth',2)
plot((DATA_HOM(:,5)),DATA_HOM(:,4),'Color',[180, 180, 180]/255,'LineWidth',2)

% plot(curve1,'k')
% plot(curve2,'r')
% plot(curve3,'b')

axis([6 40 2 7])
title('Distance Traveled vs Robot/Task Count')
xlabel('Robot/Task Count')
ylabel('Ave. Distance Traveled per Robot')
legend('heterogeneous','mixed','homogeneous')
hold off