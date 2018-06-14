function line = GetLine(x1, x2)
%Mac Schwager, MIT, 2006
% returns a 3 vector representing the normal and constant
% of the line defined by the two points x1 and x2
% so that n'[x; y] + a = 0

n = x2-x1;
n = [-n(2); n(1)];
a = -n'*x1;
line = [n; a]/norm(n);
