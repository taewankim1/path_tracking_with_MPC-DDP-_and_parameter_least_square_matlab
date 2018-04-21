function tirePlot
clear all; close all; clc;
alpha = -deg2rad(0):0.001:deg2rad(10);
% Vehicle parameter
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;
FzF = 1 * lr * m * g / ( 2 * ( lf + lr ) );
FzR = 1 * lf * m * g / ( 2 * ( lf + lr ) );
% Tire parameter
Cf = 50000;
% Cf = 70369;
% Cr = 50000;
% Mu = 3.4325;
Mu = 0.8;
% delMat = [Cf;Cf^2/Mu;Cf^3/Mu^2];
delMat = [Cf;Cf^2/Mu;Cf^3/Mu^2];

% Mu = ( delMat1(1)^2 / delMat1(2) + delMat2(1)^2 / delMat2(2) ) / 2;
% delMat = delMat2; FzF = FzR;
% Linear
FL = Cf * alpha;
% Brush
vecIni
FB1 = myBrush(delMat,alpha,FzF);
delMat2 = [78447;6.0844e9;3.6148e14];
FB2 = myBrush(delMat2,alpha,FzF);

% FB2 = myBrush(delMatTrue,alpha,FzF);
% Pacejka
By = 0.22 + ( 5200 - FzF ) / 40000;
Cy = 1.26 - ( FzF - 5200 ) / 32750;
Dy = -0.00004*FzF^2 + 1.0526 * FzF - 22.73;
Ey = -1.6;
Shy = 0;
Svy = 0;
Phiy = (1-Ey) .* (rad2deg(alpha) + Shy) + Ey / By * atan(By*(rad2deg(alpha)+Shy));
FP = Dy .* sin(Cy*atan(By*Phiy))+Svy;
%% plot
lW = 1.1;
fS = 18;
cMap = colormap('lines');
plot(rad2deg(alpha),FP,'color',cMap(4,:),'linewidth',lW), hold on
plot(rad2deg(alpha),FB1,'color',cMap(1,:),'linewidth',lW)
plot(rad2deg(alpha),FB2,'color',cMap(2,:),'linewidth',lW)
leg = legend('True(pacejka)','Initial estimation','After Learning');
set(leg,'fontsize',fS), legend boxoff
grid on
xlabel('Slip angle[deg]')
ylabel('Lateral force[N]')
set(gca,'fontsize',fS)
axis([rad2deg(alpha(1)) rad2deg(alpha(end)) -0 5000])
% plot([rad2deg(slipMax) rad2deg(slipMax)],[0 6000],'--','color',cMap(7,:));
end

function [FB,slipMax] = myBrush(delMat,alpha,FzF)
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

slipMax = atan( 3 * FzF / ( Cf_mu ) );

end






