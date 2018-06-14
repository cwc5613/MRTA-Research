function phip = Measure(p)
%Mac SChwager, MIT, 2006
% Returns the value of phi at point p
global alpha beta gamma offset;

m = length(alpha);
z = sqrt((p(1) - gamma(:,1)).^2 + (p(2) - gamma(:,2)).^2).*beta.^-1;
g = beta.^-1/sqrt(2*pi).*exp(-.5*z.^2);
phip = alpha'*g + sum(offset);
