function [newPath2]=replanPath(path1,path2,colPoint,pos)
% changeable parameter
UAVsize=1;
colPoint(1)=colPoint(1)+4*UAVsize;
colPoint(2)=colPoint(2)+4*UAVsize;
colPoint(3)=colPoint(3)+4*UAVsize;
% colTime=colLength/Vlimit*Vcoef;
newPath2=[path2(:,1:pos-1)';colPoint;path2(:,pos+1:end)']';
end

% This function change the path around the collision point



