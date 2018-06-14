function [X,mu,sigma] = MULTI_MAP(Q,R,X,mu,sigma)

global tasks
global mumap_vec

xT = tasks(1:size(tasks,2)/2);%tasks' x positions
yT = tasks(size(tasks,2)/2+1:size(tasks,2));%tasks y positions
xR = X(1,:); %robots' x positions
yR = X(2,:); %robots' y positions

mu = [xT;yT];

m = length(xT);
n = length(xR);

V = (randn(1,1)*chol(R))';

y = zeros(m,1);
for j = 1:m
    for i = 1:n
        y(j,i) = norm(mu(:,j)-X(:,i)) + V;
    end
end

A = eye(2*m);

for j = 1:m
    for i = 1:n
        den = (mu(1,j)-xR(i))^2 + (mu(2,j)-yR(i))^2;
        C(j,2*j-1:2*j,i) = [(yR(i)-mu(2,j))/den,...
            (mu(1,j)-xR(i))/den];
    end
end

% Predict
mu_pred = mu;
sig_pred = A*sigma*A' + Q;

% Compute Kalman Gain and Innovation Term

for i = 1:n
    Kt(:,:,i) = sig_pred*C(:,:,i)'/(C(:,:,i)*sig_pred*C(:,:,i)' + R*eye(m));
end

for j = 1:m
    for i = 1:n
        y_hat(j,i) = norm(mu_pred(:,j)-X(:,i));
    end
end

for i = 1:n
    inno(:,i) = Kt(:,:,i)*(y(:,i) - y_hat(:,i));
    inno_sized = zeros(2,m);
    for j = 1:m
        inno_sized(1,j,i) = inno(2*j-1,i);
        inno_sized(2,j,i) = inno(2*j,i);
    end
end

% Update and Store Values
for i = 1:n
    mu(:,:,i) = mu_pred + inno_sized(:,:,i);
    sigma(:,:,i) = sig_pred - Kt(:,:,i)*C(:,:,i)*sig_pred;
end

mu = sum(mu,3)/n;
sigma = sum(sigma,3)/n;

mumap_vec = [mumap_vec; mu];

tasks = [mu(1,:),mu(2,:)];

end
