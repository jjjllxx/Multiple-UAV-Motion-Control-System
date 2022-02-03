function [colFlag,colTime] = timeCol(trajectory1,trajectory2,colLimit)
colFlag=0;
colLimit=colLimit^2;
colTime=-1;
tLimit=min(size(trajectory1,2),size(trajectory2,2));
for t=1:tLimit
    if sum((trajectory1(:,t)-trajectory2(:,t)).^2)<colLimit
        colFlag=colFlag+1;
        colTime=[colTime,t];
    end
end
end

