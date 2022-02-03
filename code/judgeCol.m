function [flag,loc,collisionPoint] = judgeCol(path1,path2,collisionLimit)
seg1=size(path1,2)-1;seg2=size(path2,2)-1;
flag=0;collisionPoint=[-1,-1,-1];loc=0;
for i=1:seg1
    for j=1:seg2
        [d,colPoint]=calMinDistance(path1(:,i)',path1(:,i+1)',path2(:,j)',path2(:,j+1)');
        if d<collisionLimit
            flag=j;
            loc=i;
            collisionPoint=colPoint;
        end  
    end
end
end
% for 2 path, judge whether there is collision