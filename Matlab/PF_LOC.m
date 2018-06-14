function [mu,sigma] = PF_LOC(M,m,n,Q,R,X,Eq,dt,mu,sigma,tasks)

global mu_vec

xT = tasks(1,:);%tasks' x positions
yT = tasks(2,:);%tasks y positions
xR = X(1:n); %robots' x positions
yR = X(n+1:2*n); %robots' y positions

m_set = [xT;yT];

W = (randn(1,2)*chol(Q))';
V = (randn(1,1)*chol(R))';

for i = 1:n
    xR(i) = xR(i) + Eq(2*i-1)*dt + W(1);
    yR(i) = yR(i) + Eq(2*i)*dt + W(2);
end

for i = 1:m
    for j = 1:n
        y(i,j) = norm([xT(i);yT(i)]-mu(:,j)) + V;
    end
end

for m = 1:M
    for i = 1:n
        xt(:,i,m) = mu(:,i)+chol(sigma(:,:,1))*randn(2,1);
        xtm(:,i,m) = [xt(1,i,m) + Eq(2*i-1)*dt+ W(1);...
            xt(2,i,m) + Eq(2*i-1)*dt + W(2)];
        for j = 1:size(xT,2)
            yi(j,1) = norm([xT(j);yT(j)]-xtm(1:2,i,m));
        end
        wm(m,i) = exp(-0.5*(y(m,i)-yi(m))'/R*(y(m,i)-yi(m)));
    end
end

wm = wm./sum(wm);
for i = 1:size(xtm,2)
    mubar(:,i,:) = sum(xtm(:,i,:).*wm(:,i)',2);
end
mubar = sum(mubar,3)/M;

for m = 1:M
    for i = 1:n
        sigbar(:,:,m,i) = wm(m,i)*(xt(:,i,m)-mubar(:,i))*(xt(:,i,m)-mubar(:,i))';
    end
end
sigbar = squeeze(sum(sigbar,3)/n);

mu = mubar;
sigma = sigbar;

mu_vec = [mu_vec; mu];
t = toc;
end