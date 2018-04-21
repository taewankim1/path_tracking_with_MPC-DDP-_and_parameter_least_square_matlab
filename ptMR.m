function [x,u,delM,alpha] = ptMR(delT,simTime,xDes,delMat,delMatTrue,dataLimit)

% MPC parameter
horizon = 20;
% state,input matrices
% t = 0:simTime/delT;
x = zeros(6,simTime / delT);
u = zeros(2,simTime / delT);
alpha = zeros(2, simTime / delT);
x0 = [0;0;10;0;0;0];
u0 = zeros(2,horizon);
hMat = [];
tMat = [];
delM = [];
delMat0 = delMat;
delMatC = delMat0;
% MPC start
for i = 1 : simTime/delT
    fprintf('MR - current time = %d\n',i*delT)
    xGoal = xDes(1:5,i:i+horizon);
    if i == 1
        xCu = x0;
        [~,uR] = ptDDP(xCu,u0,delT,xGoal,delMatC);
    else
        xCu = xTemp;
        [~,uR] = ptDDP(xCu,uR,delT,xGoal,delMatC);
    end
    uTemp = uR(:,1);
    u(:,i) = uTemp;
    % Dynamics
    if xCu(2) > 200
       delMatReal = delMatTrue; 
    else
        delMatReal = delMat;
    end
        [xTemp,fTemp,alphaF,alphaR] = ptDynBrush(xCu,u(:,i),delT,delMatReal);
%     [xTemp,fTemp,alphaF,alphaR] = ptDynPacejka(xCu,u(:,i),delT);
    x(:,i) = xTemp;
    alpha(:,i) = [alphaF;alphaR];
    % noise
    xN = xCu + 0.0*[randn(1,1);randn(1,1);randn(1,1);0;0;randn(1,1)];
    uN = u(:,i) + 0.0*randn(2,1);
    fN = fTemp + 0.0*randn(6,1);
    % Data
    [hMat,tMat] = tireDataBrush(xN,uN,fN,hMat,tMat);
    % Linear regression
    if rem(i,50) == 0
        beta = 1000;
        if size(hMat,1) > beta && dataLimit == 1
            delMatC = pinv(hMat(end-beta+1:end,:))*tMat(end-beta+1:end,:);
        else
            delMatC = pinv(hMat)*tMat;
        end
        
        if isempty(delMatC)
            delMatC = delMat0;
        else
        end
    else
    end
    
    delM(:,i) = delMatC;
end
x = [x0,x];
delM = [delMat0,delM];
% alpha = [[0;alpha];
end

