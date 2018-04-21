function plotVC(t,u,delT,lC,fS,lW)
subplot(121)
cMap = colormap('lines');
plot(t(1:end-1)*delT,u(1,:),'-','color',cMap(lC,:),'linewidth',lW), hold on
% plot(t,t*0+xGoal(1),'--','color',cMap(2,:))
title('virtual control1')
xlabel('time(s)')
ylabel('Nm')
grid on
set(gca,'fontsize',fS)
subplot(122)
plot(t(1:end-1)*delT,u(2,:),'-','color',cMap(lC,:),'linewidth',lW), hold on
% plot(t,t*0+xGoal(1),'--','color',cMap(2,:))
title('virtual control2')
xlabel('time(s)')
ylabel('Nm')
grid on
set(gca,'fontsize',fS)
end