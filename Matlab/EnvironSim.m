function EnvironSim
global n gamma y0 tspan lw fs ms; 
%Mac Schwager, MIT, 2006
% Main function for simulating the environment.  Calls EnvironConstants to
% initialize all global variable values.  Then calls odeSimple to solve the
% differential equations.  odeSimple in turn calls EnvironDer to carry out
% each step of the differential equations.  There is an animation routine 
% inside odeSimple for visualization. Ploting functions are called after the
% ODE's are solved.

EnvironConstants;

%options = odeset('OutputSel', [1 n+1], 'outputfcn', @odeplot); 
%figure(1);
%[t y] = ode45(@EnvironDer, tspan, y0, options);

[t y] = odeSimple(@EnvironDer, tspan, y0);
f = length(t);

%plot initial voronoi config
xinit = y(1, 1:n);
yinit = y(1, n+1:2*n);
xinit = [xinit; -xinit; 2-xinit; xinit; xinit];
yinit = [yinit; yinit; yinit; -yinit; 2-yinit];
figure(2);
Voronoilw(xinit, yinit);
hold on;
plot(gamma(:, 1), gamma(:, 2), 'rx', 'LineWidth', lw, 'MarkerSize', ms);
set(gca,'LineWidth', lw, 'FontSize', fs, 'FontWeight', 'bold');
hold off;
axis([0 1 0 1]);

%plot agent paths
figure(3);
for i = 1:n
    plot(y(f,i), y(f,i+n), 'o', 'LineWidth', lw, 'MarkerSize', ms);
    hold on;
    plot(y(:,i), y(:,i+n), '--', 'LineWidth', lw, 'MarkerSize', ms);
end
plot(gamma(:,1), gamma(:,2), 'rx', 'LineWidth', lw, 'MarkerSize', ms);
set(gca,'LineWidth', lw, 'FontSize', fs, 'FontWeight', 'bold');
hold off;
axis([0 1 0 1]);

%plot final voronoi configuration
figure(4);
xfinal = y(f, 1:n);
yfinal = y(f, n+1:2*n);
xfinal = [xfinal; -xfinal; 2-xfinal; xfinal; xfinal];
yfinal = [yfinal; yfinal; yfinal; -yfinal; 2-yfinal];
Voronoilw(xfinal, yfinal)
hold on;
plot(gamma(:,1), gamma(:,2), 'rx', 'LineWidth', lw, 'MarkerSize', ms);
set(gca,'LineWidth', lw, 'FontSize', fs, 'FontWeight', 'bold');
hold off;
axis([0 1 0 1]);

%plot time vs position for agent 1
figure(5);
plot(t, y(:,1), '-', t, y(:,2), '--', 'LineWidth', lw, 'MarkerSize', ms);
set(gca,'LineWidth', lw, 'FontSize', fs, 'FontWeight', 'bold');
legend('x-coord', 'y-coord');
