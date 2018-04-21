function [hMat1,tMat1,upIdx] = tireDataBrush(state,input,f,hMat,tMat)
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;

% state
vx = state(3,:,:);
vy = state(4,:,:);
% yaw = state(5,:,:);
yawDot = state(6,:,:);
yDDot = f(4,1);
% yawDot = f(5,1);
yawDDot = f(6,1);

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
TanF = abs(tan(alphaF));
TanR = abs(tan(alphaR));


% tire vertical force
FzF = 1 * lr * m * g / ( 2 * ( lf + lr ) );
FzR = 1 * lf * m * g / ( 2 * ( lf + lr ) );

upIdx = 0;

% abs + tan ( alpha ) / sign(alpha)
if TanF > 0.001 && TanR > 0.001 && TanF < 0.2 && TanR < 0.2
    hMat1 = [hMat ; ...
        [ 2*cos(delta)*TanF*sign(alphaF), -2*cos(delta)*TanF^2 / ( 3 * FzF )*sign(alphaF), 2*cos(delta)*TanF^3 / (27 * FzF^2)*sign(alphaF) ...
        , 2*TanR*sign(alphaR), -2*TanR^2 / ( 3 * FzR )*sign(alphaR), 2*TanR^3 / (27 * FzR^2)*sign(alphaR); ...
        2 * lf *cos(delta)*TanF*sign(alphaF), -2 * lf *cos(delta)*TanF^2 / ( 3 * FzF )*sign(alphaF), 2*lf*cos(delta)*TanF^3 / (27 * FzF^2)*sign(alphaF) ...
        , -2*lr*TanR*sign(alphaR), 2*lr*TanR^2 / ( 3 * FzR )*sign(alphaR), -2*lr*TanR^3 / (27 * FzR^2)*sign(alphaR)] ...
        ];
    
    tMat1 = [ tMat ; [ m * ( yDDot + vx * yawDot); ...
        Iz * yawDDot ]];
    
    upIdx = 1;
else
    hMat1 = hMat;
    tMat1 = tMat;
    upIdx = 0;
end



end





