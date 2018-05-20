function [g, h] = InitAnimation(fig, gammae, y, t)
%Mac Schwager, MIT, 2006
% function to initialized odesimple for animation

global n tstep fs lw ms mov;

figure(fig);
time = title(strcat('time = ', num2str(t,'%.1f'), 's'));
set(time, 'EraseMode', 'xor', 'FontSize', fs);
g = plot(gammae(:,1), gammae(:,2), 'rx');
set(gca, 'FontSize', fs, 'LineWidth', lw, 'FontWeight', 'bold');
hold on;
h = plot(y(1, 1:n), y(1, n+1:2*n), 'bo');
hold off;
set(g, 'EraseMode', 'xor', 'MarkerSize', ms, 'LineWidth', lw);
set(h,'EraseMode','xor', 'MarkerSize', ms, 'LineWidth', lw);
axis([0 1 0 1]);