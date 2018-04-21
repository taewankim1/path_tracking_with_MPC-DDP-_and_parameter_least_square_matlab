function [y,f] = ptDynBrush(state,input,delT,Cf,Cr,Mu)
% parameter
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;
delMat = [Cf;Cf^2/Mu;Cf^3/Mu^2;Cr;Cr^2/Mu;Cr^3/Mu^2];

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

% tire lateral force(brush model)
linearSlipF = delMat(2) / delMat(1) * abs( tan(alphaF) ) ./ FzF;
Hf = [ abs( tan(alphaF) )', -abs( 1 * tan(alphaF).^2 / ( 3 * FzF ) )', abs( 1 * tan(alphaF ).^3 / ( 27 * FzF.^2 )  )'] ;
Lf = linearSlipF <= 3; 
Nf = linearSlipF > 3; 
% searchSlip = delMat(2) / delMat(1) * abs( tan(alphaF) ) ./ FzF;
Nfc = Mu * FzF;

if size(Hf,1) > 1
    Fcf = ( ( Hf * delMat(1:3,1) )'.*( Lf ) + Nfc .*( Nf ) ).* sign(alphaF);
else
    Fcf = ( Hf * delMat(1:3,1).*( Lf ) + Nfc .* Nf  ).* sign(alphaF);
end

linearSlipR = delMat(5) / delMat(4) * tan(alphaR) ./ FzR;
Hr = [abs( tan(alphaR) )', -abs( 1 * tan(alphaR).^2 / ( 3 * FzR ) )', abs( 1 * tan(alphaR).^3 / ( 27 * FzR.^2 ) )'] ;
if size(Hr,1) > 1
    Fcr = ( ( Hr * delMat(4:6,1) )'.*( linearSlipR <= 3  ) + Mu * FzR .*( linearSlipR > 3 ) ).* sign(alphaR);
else
    Fcr = ( Hr * delMat(4:6,1).*( linearSlipR <= 3  ) + Mu * FzR .*( linearSlipR > 3 ) ).* sign(alphaR);
end
% Fcf = delMat(1) * alphaF;
% Fcr = delMat(4) * alphaR;


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
