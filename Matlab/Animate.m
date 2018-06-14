function Animate(fig, g, h, gammaet, yt, ti)
%Mac Schwager, MIT, 2006
% carries out one animation set for odesimple

global n mov frame;

figure(fig);
title(strcat('time = ', num2str(ti,'%.1f'), 's'));
set(g, 'XData', gammaet(:,1), 'YData', gammaet(:,2));
set(h,'XData', yt(1:n), 'YData', yt(n+1:2*n));
drawnow;
if mov
    name = ['./Movies/Frames/' num2str(frame) '.jpg'];
    print('-djpeg', '-r100', name);
    frame = frame+1;
end