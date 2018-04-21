function [delMat,Idx,P1,P2] = tireDataBrushRl(state,input,f,delMatC,PInitial,P1,P2,Idx,lambda)
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;

% state
vx = state(3,:,:);
vy = state(4,:,:);
yawDot = state(6,:,:);
yDDot = f(4,1);
yawDDot = f(6,1);

% input / tire longitudinal force
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

% Recursive least square matrix
% unknown parameter
theta1 = delMatC(1:3);
theta2 = delMatC(4:6);
% regression vector
psi1 = 2 * ( lr + lf ) * cos(delta) * sign(alphaF) ...
       * [TanF, -TanF^2 / ( 3 * FzF ), TanF^3 / ( 27 * FzF^2 ) ];
psi2 = - 2 * ( lr + lf ) * 1 * sign(alphaR) ...
       * [TanR, -TanR^2 / ( 3 * FzR ), TanR^3 / ( 27 * FzR^2 ) ];
% measured value
y1 = lr * m * ( yDDot + vx * yawDot) + Iz * yawDDot ;
y2 = - lf * m * ( yDDot + vx * yawDot) + Iz * yawDDot ;
% forgetting factor
% lambda = 0.95;
% abs + tan ( alpha ) / sign(alpha)
% if TanF > 0.001 && TanR > 0.001 && TanF < 0.05 && TanR < 0.05
    Idx = Idx + 1;
    err1 = y1 - psi1 * theta1;
    err2 = y2 - psi2 * theta2;
    if Idx == 1
       P1Initial = PInitial;
       P2Initial = 2 * PInitial;
       K1 = P1Initial * psi1' / ( lambda + psi1 * P1Initial * psi1' );
       K2 = P2Initial * psi2' / ( lambda + psi2 * P2Initial * psi2' );
       P1(:,:,Idx) = 1 / lambda * ( P1Initial - P1Initial * ( psi1' * psi1) * P1Initial ...
                        / ( lambda + psi1 * P1Initial * psi1') );
       P2(:,:,Idx) = 1 / lambda * ( P2Initial - P2Initial * ( psi2' * psi2) * P2Initial ...
                        / ( lambda + psi2 * P2Initial * psi2') );
    else
       P1Initial = P1(:,:,Idx-1);
       P2Initial = P2(:,:,Idx-1);
       K1 = P1Initial * psi1' / ( lambda + psi1 * P1Initial * psi1' );
       K2 = P2Initial * psi2' / ( lambda + psi2 * P2Initial * psi2' );
       P1(:,:,Idx) = 1 / lambda * ( P1Initial - P1Initial * ( psi1' * psi1) * P1Initial ...
                        / ( lambda + psi1 * P1Initial * psi1') );
       P2(:,:,Idx) = 1 / lambda * ( P2Initial - P2Initial * ( psi2' * psi2) * P2Initial ...
                        / ( lambda + psi2 * P2Initial * psi2') ); 
    end
    
    theta1 = theta1 + K1 * err1;
    theta2 = theta2 + K2 * err2;
    
    delMat = [theta1;theta2];
% else
%     delMat = delMatC;
% end




end





