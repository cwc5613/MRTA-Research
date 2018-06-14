function EKF_ERROR_PLOT(t,x,y,mu_vec)

for j = 1:size(mu_vec,2)
    for i = 1:size(mu_vec,1)/2
        xMU(i,j) = mu_vec(2*i-1,j);
        yMU(i,j) = mu_vec(2*i,j);
    end
end

t2 = linspace(0, length(t), size(xMU,1));

tru = [x;y];
est = [xMU;yMU];

n = size(x,2);
col = repmat([1,2],1,n);

for i = 1:2*n
    subplot(2*n,col(i),i);...
        hold on;...
        plot(t,tru(:,1),'k','LineWidth',2);...
        plot(t2,xMU(:,1),'r--');...
        hold off
end

end