%% MPC -> Linear regression -> MR -> virtual control
% Do you think it will works?
% ktw
clear all; close all; clc;
simTime = 20;
% vehicle parameter
vecIni
% path generation
[xDes,uDes] = ptPathGenerator(22);
figure(1)
plot(xDes(1,:),xDes(2,:))
grid on
axis equal
%% MPC parameter
vs = 100; % Hz
delT = 1 / vs;
t = 0:delT:simTime;
% [x1,u1] = ptMPC(delT,simTime,xDes(1:5,:),delMatTrue,delMatTrue);
[x1,u1] = ptMPC(delT,simTime,xDes(1:5,:),delMat,delMatTrue);

% Model-based reinforcement learning
% [x1,u1,delM1,alpha] = ptMR(delT,simTime,xDes(1:5,:),delMat,delMatTrue,0);
% [x2,u2,delM2,alpha] = ptMR(delT,simTime,xDes(1:5,:),delMat,delMatTrue,1);
% [x3,u3,delM3,Idx] = ptMRRL(delT,simTime,xDes(1:5,:),delMat,delMatTrue);
%%
% [xDes,uDes] = ptPathGenerator(simTime);
lW = 1.1;
fS = 18;
% load x1; load u1;
% load x2; load u2;
load x3; load u3;
figure(1)
cMap = colormap('lines');
plot(xDes(1,:),xDes(2,:),'--','color',cMap(5,:),'linewidth',lW), hold on
plot(x1(1,:),x1(2,:),'color',cMap(1,:),'linewidth',lW)
% plot(x2(1,:),x2(2,:),'color',cMap(2,:),'linewidth',lW)
plot(x3(1,:),x3(2,:),'color',cMap(2,:),'linewidth',lW)
leg = legend('Desired Path','Planning only','Model-based RL');
% set(leg,'Location','Best','fontsize',fS), 
legend boxoff
axis equal
xlabel('X[m]')
ylabel('Y[m]')
grid on
set(gca,'fontsize',fS)
plot([-10 120],[45 45],'--','color',[0 0 1])
axis([-10 120 -10 140])
% axis equal
set(gcf,'units', 'pixels', 'pos',[100 100 800 600])
carMovie(x1,u1,x3,u3,simTime,delT);
% %%
% figure(2)
% subplot(121)
% plot(t(1:end-1),u2(1,:),'color',cMap(1,:),'linewidth',lW)
% title('accel')
% xlabel('time[s]')
% ylabel('Acceleration[m/s^2]')
% grid on
% set(gca,'fontsize',fS)
% subplot(122)
% plot(t(1:end-1),u2(2,:),'color',cMap(1,:),'linewidth',lW)
% title('steering')
% xlabel('time[s]')
% ylabel('Steering angle[rad]')
% grid on
% set(gca,'fontsize',fS)
% %% plot delMat
%%
figure(3)
delM = delM1;
subplot(1,3,1)
plot(t,delM(1,:)), hold on
plot(t,t*0+delMatTrue(1),'--','linewidth',lW)
title('C_f')
xlabel('time[s]')
grid on
set(gca,'fontsize',fS)
subplot(1,3,2)
plot(t,delM(2,:)), hold on
plot(t,t*0+delMatTrue(2),'--','linewidth',lW)
title('C_f^2 / \mu')
xlabel('time[s]')
grid on
set(gca,'fontsize',fS)
subplot(1,3,3)
plot(t,delM(3,:)), hold on
plot(t,t*0+delMatTrue(3),'--','linewidth',lW)
title('C_f^3 / \mu^2')
xlabel('time[s]')
grid on
set(gca,'fontsize',fS)
% subplot(2,3,4)
% plot(t,delM(4,:)), hold on
% plot(t,t*0+delMatTrue(4),'--','linewidth',lW)
% title('C_r')
% xlabel('time[s]')
% grid on
% set(gca,'fontsize',fS)
% subplot(2,3,5)
% plot(t,delM(5,:)), hold on
% plot(t,t*0+delMatTrue(5),'--','linewidth',lW)
% title('C_r^2 / \mu')
% xlabel('time[s]')
% grid on
% set(gca,'fontsize',fS)
% subplot(2,3,6)
% plot(t,delM(6,:)), hold on
% plot(t,t*0+delMatTrue(6),'--','linewidth',lW)
% title('C_r^3 / \mu^2')
% xlabel('time[s]')
% grid on
% set(gca,'fontsize',fS)
% %% tire model result
% % figure(4)
% % tirePlotResult(delMatTrue,delM(:,end))
% %% slip angle
% figure(5)
% subplot(121)
% plot(t(1:end-1),rad2deg( alpha(1,:) ),'color',cMap(1,:),'linewidth',lW)
% title('Front tire slip angle ')
% xlabel('time[s]')
% ylabel('Angle[deg]')
% grid on
% set(gca,'fontsize',fS)
% subplot(122)
% plot(t(1:end-1),rad2deg( alpha(2,:) ),'color',cMap(1,:),'linewidth',lW)
% title('Rear tire slip angle')
% xlabel('time[s]')
% ylabel('Angle[deg]')
% grid on
% set(gca,'fontsize',fS)

