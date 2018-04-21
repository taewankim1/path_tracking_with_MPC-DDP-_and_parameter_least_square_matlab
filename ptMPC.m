function [x,u,xM] = ptMPC(delT,simTime,xDes,delMat,delMatTrue)

% MPC parameter
horizon = 20;
% state,input matrices
% t = 0:simTime/delT;
x = zeros(6,simTime / delT);
u = zeros(2,simTime / delT);
x0 = [0;0;10;0;0;0];
xCu = x0;
u0 = zeros(2,horizon);
xM = [];
% delMatReal = 50000*ones(2,1);
% MPC start
for i = 1 : simTime/delT
    fprintf('MPC - current time = %d\n',i*delT)
    % way point generation
%     c = mm(xDes(1:2,:),xCu(1:2));
%     [~,k] = min( sum( c.^2,1) );
    xGoal = xDes(1:5,i:i+horizon);
    
    if i == 1
        xCu = x0;
        [~,uR] = ptDDP(xCu,u0,delT,xGoal,delMat);
    else
        xCu = xTemp;
        [xR,uR] = ptDDP(xCu,uR,delT,xGoal,delMat);
    end
    uTemp = uR(:,1);
    u(:,i) = uTemp;
    % Dynamics
    if xCu(2) > 45
        delMatReal = delMatTrue;
    else
        delMatReal = delMat;
    end
    xTemp = ptDynBrush(xCu,u(:,i),delT,delMatReal);
    x(:,i) = xTemp;
    
end
x = [x0,x];

end

function c = mm(a,b)
c = bsxfun(@minus,a,b);
end