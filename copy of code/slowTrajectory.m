function [key_points, trajectory] = slowTrajectory(key_points,colTime)
point_num=size(key_points,2)-1;
pos=2;up=size(colTime,2);
% calculate coefficient
coef=zeros(12,point_num);
for i=1:point_num
    while pos<=up && colTime(pos)<key_points(1,i+1)
        pos=pos+1;
    end
    key_points(1,i+1)=key_points(1,i+1)+pos-2;
    T=key_points(1,i+1)-key_points(1,i);
    for j=0:2
        a=key_points(j+2,i);
        b=key_points(j+2,i+1);
        C=[0 0 0 1;T^3 T^2 T 1;0 0 1 0;3*T^2 2*T 1 0];
        D=[a;b;0;0];
        coef(j*4+1:j*4+4,i)=C\D;
    end
end

path_num=key_points(1,point_num+1);
% calculate location of each second
coefX=coef(1:4,:);
coefY=coef(5:8,:);
coefZ=coef(9:12,:);
trajectory=zeros(3,path_num);
part=1;
for i=1:path_num
    if i>key_points(1,part+1)
        part=part+1;
    end
    T=i-key_points(1,part);
    trajectory(1,i)=sum(coefX(:,part).*[T^3;T^2;T;1]);
    trajectory(2,i)=sum(coefY(:,part).*[T^3;T^2;T;1]);
    trajectory(3,i)=sum(coefZ(:,part).*[T^3;T^2;T;1]);
end
end

