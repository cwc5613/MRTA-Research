function EKF_PLOT(mu_vec)

for j = 1:size(mu_vec,2)
    for i = 1:size(mu_vec,1)/2
        xMU(i,j) = mu_vec(2*i-1,j);
        yMU(i,j) = mu_vec(2*i,j);
    end
end

for i = 1:size(xMU,2)
    plot(xMU(:,i),yMU(:,i),'--')
end

end