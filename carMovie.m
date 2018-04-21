function carMovie(x1,u1,x2,u2,simTime,delT)
% prepare the visualization window and graphics callback
% set(gcf,'name','car parking','Menu','none','NumberT','off')
% set(gca,'xlim',[-30 30],'ylim',[-5 40],'DataAspectRatio',[1 1 1])
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

handles = []; handles1 = []; handles2 = []; handles3 = [];
fig = figure(1);
writeObj = VideoWriter('threeCar.avi');
writeObj.FrameRate = 20;
open(writeObj);

for i=1:10:simTime/delT
%    set(0,'currentfigure',10);
   delete(handles)
%    delete(handles1)
   delete(handles2)
%    delete(handles3)
   handles = carPlot(x1(:,i), u1(:,i),1);
   handles2 = carPlot(x2(:,i), u2(:,i),2);
%    handles3 = carPlot(x3(:,i), u3(:,i),4);
   drawnow    
   writeVideo(writeObj,getframe(fig));
end

close(writeObj);
end

