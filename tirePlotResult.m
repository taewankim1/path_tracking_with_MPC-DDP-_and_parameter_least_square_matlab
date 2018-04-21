function tirePlotResult(delMat1,delMat2)
alpha = -deg2rad(0):0.001:deg2rad(15);
% Vehicle parameter
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;
FzF = 1 * lr * m * g / ( 2 * ( lf + lr ) );
FzR = 1 * lf * m * g / ( 2 * ( lf + lr ) );
% Brush
% vecIni
delSize = size(delMat2,2);
Ff1 = myBrush(delMat1(1:3),alpha,FzF);
Fr1 = myBrush(delMat1(4:6),alpha,FzR);
% for i = 1 :100: delSize
Ff2 = myBrush(delMat2(1:3),alpha,FzF);
Fr2 = myBrush(delMat2(4:6),alpha,FzR);
% end

%% plot
lW = 1.2;
fS = 18;
subplot(1,2,1)
cMap = colormap('lines');
plot(0,0,'--','color',cMap(1,:),'linewidth',lW), hold on
plot(0,0,'color',cMap(2,:),'linewidth',lW)
leg = legend('True','Learned');
% for i = 1:100:delSize
plot(rad2deg(alpha),Ff2,'color',cMap(2,:),'linewidth',lW)
% set(h, 'color', [cMap(2,:) 1]);
% end
plot(rad2deg(alpha),Ff1,'--','color',cMap(1,:),'linewidth',lW)
set(leg,'fontsize',fS), legend boxoff
grid on
title('Front tire')
xlabel('Slip angle[deg]')
ylabel('Lateral force[N]')
set(gca,'fontsize',fS)
axis([rad2deg(alpha(1)) rad2deg(alpha(end)) -0 2000])
subplot(1,2,2)
plot(0,0,'--','color',cMap(1,:),'linewidth',lW), hold on
plot(0,0,'color',cMap(2,:),'linewidth',lW)
leg = legend('True','Learned');
% for i = 1:100:delSize
plot(rad2deg(alpha),Fr2,'color',cMap(2,:),'linewidth',lW)
% set(h, 'color', [cMap(2,:),1]);
% end
plot(rad2deg(alpha),Fr1,'--','color',cMap(1,:),'linewidth',lW)
set(leg,'fontsize',fS), legend boxoff
grid on
title('Rear tire')
xlabel('Slip angle[deg]')
ylabel('Lateral force[N]')
set(gca,'fontsize',fS)
axis([rad2deg(alpha(1)) rad2deg(alpha(end)) -0 2000])
end

function FB = myBrush(delMat,alpha,FzF)
Mu = delMat(1)^2 / delMat(2);
Cf_mu = delMat(2) / delMat(1);
Tan = tan(alpha);
FBL = 1 * ( delMat(1) * abs(Tan) ...
     - 1 / 3 * ( delMat(2) * abs(Tan).^2 / FzF )  ...
     + 1 / 27 * ( delMat(3) * abs(Tan).^3 /  FzF^2  ) ) .* sign(alpha);
LR = ( Cf_mu * abs(Tan) / ( FzF) < 3 );
NR = ( Cf_mu * abs(Tan) / ( FzF) >= 3 );
FBN = Mu .* FzF ;
FB = FBL .* LR + FBN.*NR .* sign(alpha);


end






