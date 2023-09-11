function [] = getGIFTra(world,trajectory,azimuth,elevation)
n=size(trajectory,2);
length_limit=zeros(1,n);

% record the finish time for each trajectory

for t=1:n 
    length_limit(1,t)=size(trajectory{t},2);
end
time_limit=max(length_limit)+2;

figure
for pic_num=1:time_limit
    for i=1:n  
        now=trajectory{i};
        pos=min(pic_num,length_limit(i));
        plot3(now(1,pos),now(2,pos),now(3,pos),'o','MarkerSize',10)
        hold on
        plot3(now(1,:),now(2,:),now(3,:),'LineWidth',2)
        hold on
        for ob=1:world.NumObstacles
            [X,Y,Z] = sphere(10);
            X = (X*world.radius(ob));
            Y = (Y*world.radius(ob));
            Z = (Z*world.radius(ob));
            surf(X+world.cx(ob),Y+world.cy(ob),Z+world.cz(ob),'FaceAlpha',0.2);
            colormap([0.5 0.2 0.3]);
            hold on
        end
        axis([0,100 0,100 0,100])
        hold on
    end 
%     d_angle=pic_num*360/time_limit-37.5;
%     view(d_angle,67.5-d_angle);
    view(azimuth, elevation);
    pause(0.5)
    drawnow;%画gif动图的时候这个外面的大for循环就存在，直接把以下和for*******以上的代码放到对应的位置即可
    

    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    
    if pic_num == 1
        imwrite(I,map,['tra\',num2str(azimuth),'\',num2str(elevation),'.gif'],'gif','Loopcount',inf,'DelayTime',0.5);
    else
        imwrite(I,map,['tra\',num2str(azimuth),'\',num2str(elevation),'.gif'],'gif','WriteMode','append','DelayTime',0.5);
    end
    
    hold off
end


end


% for a trajectory, get a gif and saved as local file.