function ydot = EnvironDer(t,y)
global n k; 
%Mac Schwager, MIT, 2006 
%Implements one step of the equations of motion

posx = y(1:n);
posy = y(n+1:2*n);

posxdot = zeros(n,1);
posydot = zeros(n,1);
for i = 1:n
    p = [posx(i); posy(i)]; %Get position of agent i
    [V, Ni] = Voronoi([posx posy], p); %Decentralized Voronoi
    [Mv, Lv] = CentroidNumerical(V); %Calculate centrois of Voronoi region using numerical method
    posxdot(i,1) = k*(Lv(1)/Mv - posx(i)); %Implement control law (x-component)
    posydot(i,1) = k*(Lv(2)/Mv - posy(i)); %Implement control law (y-component)
end
ydot = [posxdot; posydot];
