function [hMat1,tMat1] = tireDataLinear(x,u,f,hMat,tMat)
m = 1500;
lf = 1.2;
lr = 0.8;
tw = 1.2;
Iz = 2500;
g = 9.81;
% Fz1 = lr * m * g / ( 2 * ( lf + lr ) );
% Fz2 = lr * m * g / ( 2 * ( lf + lr ) );
% Fz3 = lf * m * g / ( 2 * ( lf + lr ) );
% Fz4 = lf * m * g / ( 2 * ( lf + lr ) );

% state
vx = x(3,1);
vy = x(4,1);
yawDot = x(6,1);

% stateDot
% xDDot = f(3,1);
yDDot = f(4,1);
yawDDot(1,1) = f(6,1);

% tire lateral velocity
vy1(1,1) = vy + lf * yawDot;
vy2(1,1) = vy + lf * yawDot;
vy3(1,1) = vy - lr * yawDot;
vy4(1,1) = vy - lr * yawDot;

% tire longitudinal velocity
vx1 = vx - tw/2*yawDot;
vx2 = vx + tw/2*yawDot;
vx3 = vx - tw/2*yawDot;
vx4 = vx + tw/2*yawDot;

% tire slip angle
alpha1 = - vy1/vx1;
alpha2 = - vy2/vx2;
alpha3 = - vy3/vx3;
alpha4 = - vy4/vx4;

% abs + tan ( alpha ) / sign(alpha)
hMat1 = [hMat ; [ alpha1+alpha2 ...
    ,alpha3+alpha4 ]; ...
    [ lf * ( alpha1+alpha2 ) ...
    ,-lr * ( alpha3+alpha4 ) ] ];
tMat1 = [ tMat ; [ m * ( yDDot + vx * yawDot); ...
    Iz * yawDDot - tw/2 * ( -u(1) + u(2) - u(3) + u(4) )]];



end





