function [inflection_points,coefficient,trajectory] = getTrajectory(path)
path_num=length(path);

% record inflection points
inflection_points=zeros(4,path_num);
inflection_num=1;
inflection_points(:,1)=[1;path(1,1:3)'];
for i=2:path_num-1
    if abs(path(i-1,2)-path(i,2)-path(i,2)+path(i+1,2))>0.2 && ~ismember(i-1,inflection_points)
        inflection_num=inflection_num+1;
        inflection_points(:,inflection_num)=[i;path(i,1);path(i,2);path(i,3)];
    end
end
inflection_points(:,inflection_num+1)=[path_num;path(path_num,1:3)'];
inflection_points=inflection_points(:,1:inflection_num+1);

% calculate coefficient
coefficient=zeros(12,inflection_num);
for i=1:inflection_num
    T=inflection_points(1,i+1)-inflection_points(1,i);
    for j=0:2
        a=inflection_points(j+2,i);
        b=inflection_points(j+2,i+1);
        C=[0 0 0 1;T^3 T^2 T 1;0 0 1 0;3*T^2 2*T 1 0];
        D=[a;b;0;0];
        coefficient(j*4+1:j*4+4,i)=C\D;
    end
end
% calculate location of each second
coefficientX=coefficient(1:4,:);
coefficientY=coefficient(5:8,:);
coefficientZ=coefficient(9:12,:);
trajectory=zeros(3,path_num);
part=1;
for i=1:path_num
    if i>inflection_points(1,part+1)
        part=part+1;
    end
    T=i-inflection_points(1,part);
    trajectory(1,i)=sum(coefficientX(:,part).*[T^3;T^2;T;1]);
    trajectory(2,i)=sum(coefficientY(:,part).*[T^3;T^2;T;1]);
    trajectory(3,i)=sum(coefficientZ(:,part).*[T^3;T^2;T;1]);
end
% draw trajectory
% figure
% axis([0 100 0 100 0 100])
% mycomet3(trajectory(1,:),trajectory(2,:),trajectory(3,:))

end

