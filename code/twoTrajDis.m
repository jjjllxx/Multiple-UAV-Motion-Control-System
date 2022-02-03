function [] = twoTrajDis(trajectory1,trajectory2,collision_limit)
time1=size(trajectory1,2);
time2=size(trajectory2,2);
time_limit=max(time1,time2);
dist=zeros(1,time_limit);
collision_limit=collision_limit*ones(1,time_limit);
for t=1:time_limit
    pos1=min(t,time1);
    pos2=min(t,time2);
    dist(1,t)=sqrt(sum((trajectory1(:,pos1)-trajectory2(:,pos2)).^2));
end

figure
for pic_num=1:time_limit
%     pos1=min(pic_num,time1);
%     pos2=min(pic_num,time2);
    plot(1:pic_num,dist(1:pic_num),1:pic_num,collision_limit(1:pic_num));
    axis([0,time_limit,0,100])
    pause(0.5)
end
end

