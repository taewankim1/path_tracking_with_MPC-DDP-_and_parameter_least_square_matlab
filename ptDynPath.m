function [y,f] = ptDynPath(state,input,delT,delMat)

% state
X = state(1,:,:);
Y = state(2,:,:);
yaw = state(3,:,:);

% input 
v = input(1,:,:);
w = input(2,:,:);

% dynamics
f1 = v.*cos(yaw);
f2 = v.*sin(yaw);
f3 = w;
            
f = [f1;f2;f3];

y = state + delT * f;



end
