function dynamicsTest
clear all; close all; clc;
delT = 0.01;
simTime = 10;
t = 0:delT:simTime;
vecIni

x0 = [0;0;10;0;0;0];
u0 = [0.1;-deg2rad(5)];
dimX = 6;
dimU = 2;

x  = zeros(dimX, size(t,2)-1);
u  = zeros(dimU, size(t,2)-1);

delMatLinear = [50000;50000];

for i = 1 : size(t,2)-1
    if i == 1
        x(:,i) = ptDynBrush(x0,u0,delT,delMat);
        u(:,i) = u0;
    else
        x(:,i) = ptDynBrush(x(:,i-1),u0,delT,delMat);
        u(:,i) = u0;
    end
    
end
x = [x0,x];
%%
lW = 1.1;
fS = 18;
cMap = colormap('lines');
figure(1)
plot(x(1,:),x(2,:),'linewidth',lW,'color',cMap(1,:))
grid on
xlabel('X[m]')
ylabel('Y[m]')
axis equal
%%
figure(2)
subplot(1,2,1)
plot(t,x(3,:),'linewidth',lW,'color',cMap(1,:))
grid on
xlabel('time[s]')
ylabel('vx[m/s]')
axis equal
subplot(1,2,2)
plot(t,x(4,:),'linewidth',lW,'color',cMap(1,:))
grid on
xlabel('time[s]')
ylabel('vy[m/s]')
axis equal
%%
figure(3)
plot(t,x(5,:),'linewidth',lW,'color',cMap(1,:))
grid on
xlabel('time[s]')
ylabel('yaw[rad/s]')
%% Movie

% prepare the visualization window and graphics callback
figure(1);
% set(gcf,'name','car parking','Menu','none','NumberT','off')
set(gca,'xlim',[-200 200],'ylim',[-100 100],'DataAspectRatio',[1 1 1])
grid on
% box on

% plot target configuration with light colors
handles = carPlot([0 0 0 0 0 0]', [0 0]');
fcolor  = get(handles,'facecolor');
ecolor  = get(handles,'edgecolor');
fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','b','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

handles = [];
for i=1:5:1000
%    set(0,'currentfigure',10);
   delete(handles)
   handles = carPlot(x(:,i), u(:,i));
   drawnow    
end



end

