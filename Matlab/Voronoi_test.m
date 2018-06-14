%% Voronoi Test
close all;clear all; clc;
x = rand(1,10);
y = rand(1,10);
[v, c] = voronoi(x,y);
figure;
plot(x,y,'r+',vx,vy,'b-')
axis([-1 2 -1 2])
% [v,c] = voronoin([x' y']);
% for ii = 1:length(c)
%     disp(c{ii});
% end
{Mv, Lv] =
function [Mv, Lv] = CentroidNumerical(V)
% Calculate the centroid of a region V parameterized by K'*a. Uses
% numerical descritization of region to evaluate the centroid integrals.
global res

%make rectangular domain to contain region
xmax = max(V(:,1));
xmin = min(V(:,1));
ymax = max(V(:,2));
ymin = min(V(:,2));

%integration step
xstep = (xmax-xmin)/res;
ystep = (ymax-ymin)/res;
%integration loop to calc M, intxphi, intyphi
Mv = 0;
Lv = [0 0]';
for x = xmin+xstep/2:xstep:xmax-xstep/2
    for y = ymin+ystep/2:ystep:ymax-ystep/2
        q = [x y]';
        if inpolygon(q(1), q(2), V(:,1), V(:,2))
            phiq = Measure(q);
            Mv = Mv + xstep*ystep*phiq;
            Lv = Lv + xstep*ystep*phiq*q;
        end
    end
end

if Mv == 0
    figure(1);
%     hold on;
%     plot(V(:,1), V(:,2), 'r-');
%     hold off;
    'Calculated zero mass over Voronoi region.'
end