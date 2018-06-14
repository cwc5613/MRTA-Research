% function initMRTA(n,m)
n = 5;
m = 5;

% Attributes of 3 types of robots
R1 = [1 1 1]';
R2 = [1 1 1]';
R3 = [1 1 1]';

% Requirements of 3 types of tasks
T1 = [1 1 1]';
T2 = [1 1 1]';
T3 = [1 1 1]';

Rx = [R1 R2 R3];
Tx = [T1 T2 T3];

% Each column represents attributes of robot i
for ii = 1:n
    theta(:,ii) = Rx(:,floor(3*rand(1)+1));
end

% Each column represents requirements of task j
for ii = 1:m
    psi(:,ii) = Tx(:,floor(3*rand(1)+1));
end

% Spawn locations of robots and tasks
x0 = rand(1,2*n)*10;
t0 = rand(1,2*m)*10;


