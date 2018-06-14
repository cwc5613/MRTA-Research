function plot_err(t,x,y,xMU,yMU)

figure
subplot(2,1,1)
for i = 1:size(x,2)
    hold on
    plot(t,abs(x(:,i)-xMU(:,i))/x(:,i)*100)
end
hold off;
subplot(2,1,2)
for i = 1:size(y,2)
    hold on
    plot(t,abs(y(:,i)-yMU(:,i))/y(:,i)*100)
end
hold off;
