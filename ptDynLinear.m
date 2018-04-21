function [y,f] = ptDynLinear(state,input,delT,delMat)
% parameter
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;

% tire property
Cf = delMat(1);
Cr = delMat(2);

% states
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
alpha1 = delta - vyf./vxf;
alpha2 = - vyr./vxr;

% linear tire model
Fcf = Cf * alpha1;
Fcr = Cr * alpha2;


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
