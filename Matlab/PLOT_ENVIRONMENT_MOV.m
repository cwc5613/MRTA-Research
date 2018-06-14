function M = PLOT_ENVIRONMENT_MOV(x,y,num,FeatShape,FeatColors,xt,yt,numt,FeatShapet,bot_type,task_type)

if size(x,1) < size(y,1)
    total_size = size(x,1);
else
    total_size = size(y,1);
end

for i = 1:total_size
    for j = 1:num
        ROB(i,j) = plot(x(i,j),y(i,j),strcat(FeatColors(bot_type(j)),FeatShape),'Linewidth',2);
    end
    for j = 1:numt
        TSK(i,j) = plot(xt(i,j),yt(i,j),strcat(FeatColors(task_type(i,j)),FeatShapet),'Linewidth',2);
    end
    [vx,vy] = voronoi(x(i,:),y(i,:));
    VOR = plot(vx,vy,'k');
    M(i) = getframe;
    set(VOR,'Visible','off')
    set(ROB(i,:),'Visible','off')
    set(TSK(i,:),'Visible','off')
end

end