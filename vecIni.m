% tire property
Cf = 50000;
Cr = 50000;
Mu = 0.8;
delMat = zeros(6,1);

delMat(1) = Cf;
delMat(2) = Cf^2 / Mu;
delMat(3) = Cf^3 / Mu^2;

delMat(4) = Cr;
delMat(5) = Cr^2 / Mu;
delMat(6) = Cr^3 / Mu^2;
alp = 0.25;
Cf = Cf * alp;
Cr = Cr * alp;
Mu = 0.3;
delMatTrue = zeros(6,1);

delMatTrue(1) = Cf;
delMatTrue(2) = Cf^2 / Mu;
delMatTrue(3) = Cf^3 / Mu^2;

delMatTrue(4) = Cr;
delMatTrue(5) = Cr^2 / Mu;
delMatTrue(6) = Cr^3 / Mu^2;
