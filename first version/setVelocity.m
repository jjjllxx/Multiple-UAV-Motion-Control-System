function [seg_path,seg_time,seg_a] = setVelocity(path,speed)
path_num=length(path);

% record inflection points
inflection_points=zeros(4,path_num);
inflection_num=1;
inflection_points(:,1)=[1;path(1,1:3)'];
for i=2:path_num-1
    if abs(path(i-1,2)-path(i,2)-path(i,2)+path(i+1,2))>0.3 && ~ismember(i-1,inflection_points)
        inflection_num=inflection_num+1;
        inflection_points(:,inflection_num)=[i;path(i,1);path(i,2);path(i,3)];
    end
end
inflection_points(:,inflection_num+1)=[path_num;path(path_num,1:3)'];
inflection_points=inflection_points(:,1:inflection_num+1);

seg_time=zeros(1,inflection_num);seg_path=zeros(1,inflection_num);
for i =1:inflection_num
    seg_path(i)=(sum((inflection_points(2:4,i)-inflection_points(2:4,i+1)).^2))^0.5;
    seg_time(i)=round(seg_path(i)/speed);
end

% calculate acceleration
seg_a=zeros(3,inflection_num);
for i=1:inflection_num
    seg_a(:,i)=2*(inflection_points(2:4,i+1)-inflection_points(2:4,i))/(seg_time(i).^2);
end
% generate trajectory
trajectory=zeros(3,sum(seg_time)+1);
for i=1:inflection_num
    tmp_time=0;
    for j=1:seg_time(i)
        trajectory(:,tmp_time+1)=inflection_points(2:4,i)+0.5*seg_a(:,i)*tmp_time^2;
        tmp_time=tmp_time+1;
    end
end

% draw trajectory
mycomet3(trajectory(1,:),trajectory(2,:),trajectory(3,:))

end

