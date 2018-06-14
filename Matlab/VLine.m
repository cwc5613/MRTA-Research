function Line = VLine(pi, p0);
%returns the parameters for the line [a b]*q+c = 0
%perp to pi-p0 and passing through 1/2(pi+p0).  Used for
%calculating voronoi regions.

s = pi-p0;
a = s(1);
b = s(2);
c = -1/2*(pi'*pi-p0'*p0);
Line = [a b c]';