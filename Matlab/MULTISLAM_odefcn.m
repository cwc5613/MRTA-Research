function Eq = MULTISLAM_odefcn(t,X,n,m,range,theta,Q,R,M,Q_map,R_map)
%X: 1xn vector of robot positions at previous time step with entries
%1:n being x coordinates and entries n+1:2*n being y coordinates
%n: number of robots
%m: number of tasks
%tasks: m x 2 matrix with x coordinates in first column and y
%coordinates in second column

global tasks
global psi
global mu
global mumap
global sigma
global sigmap;
global indROB;

%initialize variables
Eq = zeros(2*n, 1);
xR = X(1:n); %robots' x positions
yR = X(n+1:2*n); %robots' y positions
xT = tasks(1:size(tasks,2)/2);%tasks' x positions
yT = tasks(size(tasks,2)/2+1:size(tasks,2));%tasks y positions
RR = zeros(n, n); %inter-robot distances
RT = zeros(m, n); %task-robot distances
AR = zeros(n, n);
AT = zeros(m, n);
U = zeros(m, n);
U2 = zeros(m,n);

mu = [xR,yR]';

%take care of later
velocitySaturation = 1;
alpha = 3;
beta = 1;

%compute robot distances and form adjacency matrix
for i = 1:n
    for j = 1:n
        RR(i, j) = sqrt((xR(j)-xR(i))^2 + (yR(j)-yR(i))^2);
        if RR(i, j) <= range && i ~= j
            AR(i, j) = 1;
        end
    end
end

%compute task distances
for i = 1:m
    for j = 1:n
        RT(i, j) = sqrt((xR(j)-xT(i))^2 + (yR(j)-yT(i))^2);
        if RT(i, j) <= range
            AT(i, j) = 1;
        end
    end
end

%compute utilities
for i = 1:m
    for j = 1:n
        U(i, j) = (dot(theta(:, j), psi(:, i))^beta)/(RT(i, j)^alpha);
    end
end

%adjust utilities
for i = 1:m
    for j = 1:n
        neighbors = nodeNeighbors(AR, j);
        tempU = [];
        for k = neighbors
            tempU = [tempU U(i,k)];
        end
        if isempty(neighbors)
            tempU = inf;
        end
        U2(i, j) = U(i, j) - max(tempU);
    end
end

%apply dynamics
for j = 1:n
    [maxU, idxTask] = max(U2(:,j));
    if maxU > 0
        Eq(j) = (xT(idxTask) - xR(j));
        Eq(n+j) = yT(idxTask) - yR(j);
        velocity = sqrt((Eq(j))^2 + Eq(n+j)^2);
        %saturate velocityvelocitySaturation
        if velocity > velocitySaturation
            Eq(j) = velocitySaturation*Eq(j)/velocity;
            Eq(n+j) = velocitySaturation*Eq(n+j)/velocity;
        end
        %todo
    else
        [V, ~] = Voronoi([xR yR], [xR(j); yR(j)]); %Decentralized Voronoi
        [Mv, Lv] = CentroidNumerical(V); %Calculate centroids of Voronoi region using numerical method
        Eq(j) = (Lv(1)/Mv - xR(j)); %Implement control law (x-component)
        Eq(j+n) = (Lv(2)/Mv - yR(j)); %Implement control law (y-component)
    end
end

dt = 0.02;

[mu,sigma] = PF_LOC(M,m,n,Q,R,X,Eq,dt,mu,sigma,mumap);
[mu,mumap,sigmap] = MULTI_MAP(Q_map,R_map,mu,mumap,sigmap);

updateTasks(X, m, n)

end