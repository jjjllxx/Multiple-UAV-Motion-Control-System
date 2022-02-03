function [path,coef,trajectory] = getTra(path)
Vlimit=5;
Vcoef=1.2;
point_num=size(path,2)-1;

% calculate coefficient
time=ones(1,point_num);
for i=1:point_num
    time(i)=round(sum((path(:,i+1)-path(:,i)).^2)^0.5/Vlimit*Vcoef);
end

coef=zeros(12,point_num);
for i=1:point_num
    T=time(i);
    for j=0:2
        a=path(j+1,i);
        b=path(j+1,i+1);
        C=[0 0 0 1;T^3 T^2 T 1;0 0 1 0;3*T^2 2*T 1 0];
        D=[a;b;0;0];
        coef(j*4+1:j*4+4,i)=C\D;
    end
end
% calculate location of each second
coefX=coef(1:4,:);
coefY=coef(5:8,:);
coefZ=coef(9:12,:);
path_num=sum(time);
trajectory=zeros(3,path_num);
t=1;

for i=1:point_num
    for T=0:time(i)
        trajectory(1,t)=sum(coefX(:,i).*[T^3;T^2;T;1]);
        trajectory(2,t)=sum(coefY(:,i).*[T^3;T^2;T;1]);
        trajectory(3,t)=sum(coefZ(:,i).*[T^3;T^2;T;1]);
        t=t+1;
    end
end
% draw trajectory
% figure
axis([0 100 0 100 0 100])
mycomet3(trajectory(1,:),trajectory(2,:),trajectory(3,:))

end

% also get trajectory