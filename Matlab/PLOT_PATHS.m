function PLOT_PATHS(x,y,num,FeatShape,FeatColors,xt,yt,numt,FeatShapet,bot_type,task_type,homo)

global mu_vec

if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end
if homo == true
    for i = 1:total_size
        for j = 1:num
            plot(x(i,j),y(i,j),'k.','Linewidth',2);
        end
        for j = 1:numt
            plot(xt(i,j),yt(i,j),'rx','Linewidth',2);
        end
    end
else
    for i = 1:total_size
        for j = 1:num
            plot(x(i,j),y(i,j),strcat(FeatColors(bot_type(j)),'.'),'Linewidth',2);
        end
        for j = 1:numt
            plot(xt(i,j),yt(i,j),strcat(FeatColors(task_type(i,j)),'x'),'Linewidth',2);
        end
    end
end

EKF_PLOT(mu_vec)

end