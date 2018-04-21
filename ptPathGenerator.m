function [xDes,u,t] = ptPathGenerator(simTime)
% clear all; close all; clc;
lW = 1.1;
fS = 15;

x0 = [0;0;0];
delT = 0.01;
% simTime = 15;
T = simTime / delT;
t = 0:delT:simTime;
t1 = 0:delT:15-delT;
t2 = 15:delT:simTime-delT;
u1 = [10 + 0 * t1 ; 0.4 * sin( pi / 7.5 * t1 )]; 
u2 = [10 + 0 * t2 ; - 0.4 * sin( pi / 7.5 * t2 )];
u = [u1,u2];
for i = 1 : T
    if i == 1
        [x(:,i),f(:,i)] = ptDynPath(x0,u(:,i),delT);
    else
        [x(:,i),f(:,i)] = ptDynPath(x(:,i-1),u(:,i),delT);
    end
    
end
xData = x(:,1:end-1);
% X, Y, yaw, yawDot, vx
xDes = [x;u(2,:);u(1,:)];
% %% plot position
% figure(1)
% cMap = colormap('lines');
% plot(x(1,:),x(2,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% axis equal
% title('Position')
% xlabel('X(m)')
% ylabel('Y(m)')
% set(gca,'fontsize',fS)
% %% plot velocity
% figure()
% subplot(121)
% plot(t,x(3,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('vx(m/s)')
% set(gca,'fontsize',fS)
% subplot(122)
% plot(t,x(4,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('vy(m/s)')
% set(gca,'fontsize',fS)
% %% plot yaw
% figure()
% subplot(121)
% plot(t,x(5,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('yaw(rad)')
% set(gca,'fontsize',fS)
% subplot(122)
% plot(t,x(6,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('yaw rate(rad/s)')
% set(gca,'fontsize',fS)
% %% plot input
% figure()
% subplot(121)
% plot(t,u(1,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('tire1 input(N)')
% set(gca,'fontsize',fS)
% subplot(122)
% plot(t,u(2,:),'color',cMap(1,:),'linewidth',lW)
% grid on
% xlabel('time(s)')
% ylabel('tire2 input(N)')
% set(gca,'fontsize',fS)
% 
% 
% 
