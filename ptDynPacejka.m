function [y,f,alphaF,alphaR] = ptDynPacejka(state,input,delT)
% parameter
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;
% Mu = 0.8;
% Mu = ( delMat(1)^2 / delMat(2) + delMat(4)^2 / delMat(5) ) / 2;

% state
X = state(1,:,:);
Y = state(2,:,:);
vx = state(3,:,:);
vy = state(4,:,:);
yaw = state(5,:,:);
yawDot = state(6,:,:);

% input / tire longitudinal force
ax = input(1,:,:);
delta = input(2,:,:);

% tire lateral velocity
vyf = vy + lf * yawDot;
vyr = vy - lr * yawDot;

% tire longitudinal velocity
vxf = vx;
vxr = vx;

% tire slip angle
alphaF = delta - atan( vyf./vxf );
alphaR = - atan( vyr./vxr );

% tire vertical force
FzF = 1 * lr * m * g / ( 2 * ( lf + lr ) );
FzR = 1 * lf * m * g / ( 2 * ( lf + lr ) );

% tire lateral force(pacejka model)
Fcf = myPacejka(alphaF,FzF);
Fcr = myPacejka(alphaR,FzR);

% dynamics
f1 = vx .* cos(yaw) - vy.*sin(yaw);
f2 = vx .* sin(yaw) + vy.*cos(yaw);
f3 = vy .* yawDot + ax;
f4 = -vx .* yawDot + 2 / m * ( Fcf .* cos(delta) + Fcr );
f5 = yawDot;
f6 = 2 / Iz * ( lf * Fcf .* cos(delta) - lr * Fcr );
            
f = [f1;f2;f3;f4;f5;f6];

y = state + delT * f;

end

function FP = myPacejka(alpha, FzF)

By = 0.22 + ( 5200 - FzF ) / 40000;
Cy = 1.26 - ( FzF - 5200 ) / 32750;
Dy = -0.00004*FzF^2 + 1.0526 * FzF - 22.73;
Ey = -1.6;
Shy = 0;
Svy = 0;
Phiy = (1-Ey) .* (rad2deg(alpha) + Shy) + Ey / By * atan(By*(rad2deg(alpha)+Shy));
FP = Dy .* sin(Cy*atan(By*Phiy))+Svy;

end
