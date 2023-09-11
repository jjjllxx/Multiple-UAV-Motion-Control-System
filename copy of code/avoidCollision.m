clear;close;clc;
% size of UAV 
UAVsize=1;
Vlimit=5;
Vcoef=1.2;
colLimit=3*UAVsize;

% set two path to meet, in order to check the result
path1=[5, 40, 5; 20, 20, 30; 50, 50, 50; 60, 50, 80]';
start1=path1(:,1)';
path2=[40, 5, 5; 20, 20, 30; 40, 60, 50; 70, 70, 90]';
figure
plot3(path1(1, :),path1(2, :),path1(3, :))
hold on
plot3(path2(1, :),path2(2, :),path2(3, :))

pause(5)
% findcollisionPoint
colPos = 2;
colPoint = path2(:, colPos)';
colLength = sum((colPoint - start1) .^ 2) ^ 0.5;
slowCoef = 1 - colLimit / colLength;
% colTime=colLength/Vlimit*Vcoef;
path2(:, colPos) = path2(:, colPos) * slowCoef;

[~, coef1, tra1] = getTra(path1);

[~, coef2, tra2] = getTra(path2);

% draw two path
figure
plot3(path1(1,:),path1(2,:),path1(3,:))
hold on
plot3(path2(1,:),path2(2,:),path2(3,:))
time_limit=max(size(tra2,2),size(tra1,2));
for i=1:time_limit
    if i<size(tra1,2)
        comet3(tra1(1,i:i+1),tra1(2,i:i+1),tra1(3,i:i+1))
    end
    if i<size(tra2,2)
        comet3(tra2(1,i:i+1),tra2(2,i:i+1),tra2(3,i:i+1))
    end
    pause(0.8)
end

% mycomet3(tra1(1,:),tra1(2,:),tra1(3,:))
% mycomet3(tra2(1,:),tra2(2,:),tra2(3,:))
% axis([0 100 0 100 0 100])

% x=x0+t(x1-x0) y=y0+t(y1-y0) z=z0+t(z1-z0)
function [path, coef, trajectory] = getTra(path)
Vlimit = 5;
Vcoef = 1.2;
point_num = size(path, 2) - 1;

% calculate coefficient
time = ones(1, point_num);
for i = 1 : point_num
    time(i) = round(sum((path(:, i+1)-path(:, i)) .^ 2) ^ 0.5 / Vlimit * Vcoef);
end

coef = zeros(12, point_num);
for i = 1 : point_num
    T = time(i);
    for j = 0:2
        a = path(j+1,i);
        b = path(j+1,i+1);
        C = [0 0 0 1;T^3 T^2 T 1;0 0 1 0;3*T^2 2*T 1 0];
        D = [a;b;0;0];
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
% axis([0 100 0 100 0 100])
% mycomet3(trajectory(1,:),trajectory(2,:),trajectory(3,:))

end

