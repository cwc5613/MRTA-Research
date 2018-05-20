function [t, y] = odeSimple(F, tspan, y0)
%Mac Schwager, MIT, 2006
% very simple ODE solver with fixed time step
global n tstep gamma lw fs ms anim mov;

y = y0;
t = tspan(1);
tinit = tspan(1);
tfinal = tspan(2);
ytmo = y0';
%initialize animation
if anim
   [g, h] = InitAnimation(1, gamma, y0, t);
end
for ti = tinit:tstep:tfinal-tstep
    ydot = feval(F, ti, ytmo);
    yt = ytmo + tstep*ydot;
    ytmo = yt;
    y = [y; yt'];
    t = [t; ti+tstep];
    %animate
    if anim
        Animate(1, g, h, gamma, yt, ti);
    end
end