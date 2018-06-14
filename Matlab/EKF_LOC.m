function [mu,sigma] = EKF_LOC(m,n,Q,R,X,Eq,dt,mu,sigma)

global tasks
global mu_vec

xT = tasks(1:size(tasks,2)/2);%tasks' x positions
yT = tasks(size(tasks,2)/2+1:size(tasks,2));%tasks y positions
xR = X(1:n); %robots' x positions
yR = X(n+1:2*n); %robots' y positions

m_set = [xT;yT];

W = (randn(1,2)*chol(Q))';
V = (randn(1,1)*chol(R))';

A = [1,0;...
    0,1];

for i = 1:n
    xR(i) = xR(i) + Eq(2*i-1)*dt + W(1);
    yR(i) = yR(i) + Eq(2*i)*dt + W(2);
end

for i = 1:m
    for j = 1:n
        y(i,j) = norm([xT(i);yT(i)]-mu(:,j)) + V;
        C(i,:,j) = [(mu(1,j)-m_set(1,i))/norm(m_set(:,i)-mu(:,j)),...
            (mu(2,j)-m_set(2,i))/norm(m_set(:,i)-mu(:,j))];
    end
end

for i = 1:n
    
    % Predict
    mu_pred(1,i) = mu(1,i) + Eq(2*i-1)*dt;
    mu_pred(2,i) = mu(2,i) + Eq(2*i)*dt;
    sig_pred = A*sigma(:,:,i)*A' + Q;
    
    % Compute Kalman Gain and Innovation Term
    Kt(:,:,i) = sig_pred*C(:,:,i)'/(C(:,:,i)*sig_pred*C(:,:,i)' + R*eye(size(m_set,2)));
    
end

for i = 1:m
    for j = 1:n
        y_hat(i,j) = norm([xT(i);yT(i)]-mu_pred(:,j));
    end
end

% Update and Store Values
for i = 1:n
    mu2(:,:,i) = mu_pred + Kt(:,:,i)*(y - y_hat);
    sigma(:,:,i) = sig_pred - Kt(:,:,i)*C(:,:,i)*sig_pred;
end

mu = sum(mu2,3)/n;

mu_vec = [mu_vec; mu];

end
