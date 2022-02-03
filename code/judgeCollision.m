% 
clear;close;clc;
% parameter of UAV 
UAVsize=1;Vlimit=5;Vcoef=1.2;colLimit=3*UAVsize;

% set two path to meet, in order to check the result
path1=[5,40,5;20,20,30;50,50,50;60,50,80]';
start1=path1(:,1)';
path2=[40,5,5;20,20,30;40,60,50;70,70,90]';
path3=[35,0,0;15,15,25;35,55,45;65,65,85]';
path4=[80,80,20;60,25,60;40,30,40;10,80,10]';

figure
plot3(path1(1,:),path1(2,:),path1(3,:))
hold on
plot3(path2(1,:),path2(2,:),path2(3,:))
plot3(path3(1,:),path3(2,:),path3(3,:))
plot3(path4(1,:),path4(2,:),path4(3,:))
title('initial path')
legend('path1','path2','path3','path4')

% [flag1,cp1]=judgeCol(path1,path2,colLimit);
[flag2,~,cp2]=judgeCol(path1,path2,colLimit);
[flag3,loc1,cp3]=judgeCol(path1,path3,colLimit);
[flag4,~,cp4]=judgeCol(path1,path4,colLimit);

test1=showDistance(path1(:,loc1:loc1+1)',path3(:,flag3:flag3+1)');

if flag2
    path2=replanPath(path1,path2,cp2,flag2);
end
if flag3
    path3=replanPath(path1,path3,cp3,flag3);
end
if flag4
    path4=replanPath(path1,path4,cp4,flag4);
end

[flag2new,~,cp2new]=judgeCol(path1,path2,colLimit);
[flag3new,~,cp3new]=judgeCol(path1,path3,colLimit);
[flag4new,~,cp4new]=judgeCol(path1,path4,colLimit);

test2=showDistance(path1(:,loc1:loc1+1)',path3(:,flag3:flag3+1)');

figure
plot3(path1(1,:),path1(2,:),path1(3,:))
hold on
plot3(path2(1,:),path2(2,:),path2(3,:))
plot3(path3(1,:),path3(2,:),path3(3,:))
plot3(path4(1,:),path4(2,:),path4(3,:))
legend('path1','path2','path3','path4')
title('path after replanning')



