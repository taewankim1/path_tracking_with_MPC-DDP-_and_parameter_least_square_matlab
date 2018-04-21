function [x,u,delM,Idx] = ptMRRL(delT,simTime,xDes,delMat,delMatTrue)

% MPC parameter
horizon = 20;
% state,input matrices
% t = 0:simTime/delT;
x = zeros(6,simTime / delT);
u = zeros(2,simTime / delT);
x0 = [0;0;10;0;0;0];
u0 = zeros(2,horizon);
hMat = [];
tMat = [];
delM = [];
delMat0 = delMat;
delMatC = delMat;
% Recursive least square parameter
lambda = 0.965; P1 = []; P2 = [];
PInitial = diag( [0.2 2 2] ); Idx = 0; 
% MPC start
for i = 1 : simTime/delT
    fprintf('MR - current time = %d\n',i*delT)
    xGoal = xDes(1:5,i:i+horizon);
    if i == 1
        xCu = x0;
        [~,uR] = ptDDP(xCu,u0,delT,xGoal,delMatTrue);
    else
        xCu = xTemp;
        [~,uR] = ptDDP(xCu,uR,delT,xGoal,delMatTrue);
    end
    uTemp = uR(:,1);
    u(:,i) = uTemp;
    if xCu(2) > 45
        delMatReal = delMatTrue;
    else
        delMatReal = delMat;
    end
    % Dynamics
    [xTemp,fTemp] = ptDynBrush(xCu,u(:,i),delT,delMatReal);
    x(:,i) = xTemp;
    % noise
    xN = xCu + 0*randn(6,1);
    uN = u(:,i) + 0*randn(2,1);
    fN = fTemp + 0*randn(6,1);
    % Recursive least square
    [delMatC,Idx,P1,P2] = tireDataBrushRl(xN,uN,fN,delMatC,PInitial,P1,P2,Idx,lambda);
    delM(:,i) = delMatC;
end
x = [x0,x];
delM = [delMat0,delM];
end

