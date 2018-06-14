function [theta,x0,t0] = INIT(n,m,homo)

global Tx
global psi

val = 50;
fact = 2;

% Attributes of 3 types of robots
R1 = [val floor(fact*rand+1) floor(fact*rand+1)]';
R2 = [floor(fact*rand+1) val floor(fact*rand+1)]';
R3 = [floor(fact*rand+1) floor(fact*rand+1) val]';

% Requirements of 3 types of tasks
T1 = [val floor(fact*rand+1) floor(fact*rand+1)]';
T2 = [floor(fact*rand+1) val floor(fact*rand+1)]';
T3 = [floor(fact*rand+1) floor(fact*rand+1) val]';

if homo == true
    Rx = [R1 R1 R1];
    Tx = [T1 T1 T1];
elseif homo == false
    Rx = [R1 R2 R3];
    Tx = [T1 T2 T3];
end

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

end