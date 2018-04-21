clear all; close all; clc;
m = 1466 + 82*2;
lf = 1.071;
lr = 1.724;
Iz = 2744;
g = 9.81;

% tire vertical force
FzF = 1 * lr * m * g / ( 2 * ( lf + lr ) );
FzR = 1 * lf * m * g / ( 2 * ( lf + lr ) );

% tire slip angle
delta = deg2rad(-2);
alphaF = [-0.1:0.001:0.1]';
TanF = abs(tan(alphaF));

% regression vector
psi1Mat =[2 * ( lr + lf ) .* cos(delta) .* sign(alphaF).* TanF, ...
    2 * ( lr + lf ) .* cos(delta) .* sign(alphaF) .*-TanF.^2 / ( 3 * FzF ), ...
    2 * ( lr + lf ) .* cos(delta) .* sign(alphaF).* TanF.^3 / ( 27 * FzF^2 ) ];

Cf = 50000;
Cr = 50000;
Mu = 0.8;
delMat = zeros(3,1);

delMat(1) = Cf;
delMat(2) = Cf^2 / Mu;
delMat(3) = Cf^3 / Mu^2;

y1Mat = psi1Mat * delMat;
theta2 = pinv(psi1Mat) * y1Mat;
theta1 = [40000;40000^2/0.6;40000^3/0.6^2];
PInitial = diag([1,1,1]); P1 = []; P2 = [];
lambda = 0.8;
for Idx = 1 : size(alphaF,1)
    %     Idx = Idx + 1;
    psi1 = psi1Mat(Idx,:);
    err1 = y1Mat(Idx,1) - psi1 * theta1;
    
    if Idx == 1
        P1Initial = PInitial;
        K1 = P1Initial * psi1' / ( lambda + psi1 * P1Initial * psi1' );
        P1(:,:,Idx) = 1 / lambda * ( P1Initial - P1Initial * ( psi1' * psi1 ) * P1Initial ...
            / ( lambda + psi1 * P1Initial * psi1') );
    else
        P1Initial = P1(:,:,Idx-1);
        K1 = P1Initial * psi1' / ( lambda + psi1 * P1Initial * psi1' );
        P1(:,:,Idx) = 1 / lambda * ( P1Initial - P1Initial *  ( psi1' * psi1 ) * P1Initial ...
            / ( lambda + psi1 * P1Initial * psi1') );
    end
    
    theta1 = theta1 + K1 * err1;
    
    %     delMatEst = theta1];
    
end


