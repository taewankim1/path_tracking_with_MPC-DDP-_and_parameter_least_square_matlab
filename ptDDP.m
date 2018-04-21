function [x,u] = ptDDP(initialState,initialInput,delT,stateGoal,delMat)

full_DDP = false;

% the optimization path
DYNCST = @(state,input,i) ptCst(state,input,full_DDP,stateGoal,delT,i,delMat);

Op.lims = [-100000 100000;-deg2rad(15) deg2rad(15)];

[x,u]= myiLQG(DYNCST, initialState, initialInput, Op);



end

function lc = ptCost(state,input,stateGoal,i)
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns
if nargin == 4
   stateGoal = stateGoal(:,i);
end

final = isnan(input(1,:));
inputTemp = input;
input(:,final) = 0;

% control cost
P = diag([1,5]);
lu    = 1/2*quadCost(input,zeros(2,1),P);

% running cost
Qp = 1*diag([8,8,10,10,10]);

lx = 1/2*quadCost(state([1,2,5,6,3],:),stateGoal,Qp);


% total cost

lc = lu + lx ;

end


function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = ptCst(state,input,full_DDP,stateGoal,delT,i,delMat)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives
if nargout == 2
   f = ptDynBrush(state,input,delT,delMat);
   c = ptCost(state,input,stateGoal,i);
else
   % state and control indices
   ix = 1:6;
   iu = 7:8;
   n = iu(end);
   % dynamic first derivatives
   xu_dyn = @(xu) ptDynBrush(xu(ix,:), xu(iu,:),delT,delMat);
   J = finite_difference(xu_dyn,[state;input]);
   fx = J(:,ix,:);
   fu = J(:,iu,:);
   
   % dynamics second derivatives
   % none
   [fxx,fxu,fuu] = deal([]);
   
   % cost first derivatives
   I = repmat(i,1,n+1);
   xu_cost = @(xu) ptCost(xu(ix,:),xu(iu,:),stateGoal,I);
   J = squeeze(finite_difference(xu_cost,[state;input]));
   cx = J(ix,:);
   cu = J(iu,:);
   
   % cost second derivatives
   I2 = repmat(I,1,n+1);
   xu_cost2 = @(xu) ptCost(xu(ix,:),xu(iu,:),stateGoal,I2);
   xu_Jcst = @(xu) squeeze(finite_difference(xu_cost2,xu));
   JJ = finite_difference(xu_Jcst,[state;input]);
   JJ = 0.5*(JJ + permute(JJ,[2 1 3]));
   cxx = JJ(ix,ix,:);
   cxu = JJ(ix,iu,:);
   cuu = JJ(iu,iu,:);
   
   [f,c] = deal([]);
    
    
end



end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end
[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
end

function y = sabsGoal(x,p,goal)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(pp(x,-goal).^2,p.^2)), -p);
end

function y = hlo(p,pGoal,Qp,alpha)
% Huber-like loss function
A = tt( mm(p,pGoal).^2, diag(Qp) );
y = sqrt( sum( pp(A,alpha*ones(2,1)) ) );
end

function y = quadCost(x,xGoal,Q)
% Quadratic cost function
y = sum( tt(mm(x,xGoal).^2,diag(Q)) );
end

function c = pp(a,b)
c = bsxfun(@plus,a,b);
end
function c = tt(a,b)
c = bsxfun(@times,a,b);
end
function c = mm(a,b)
c = bsxfun(@minus,a,b);
end