function [inflec,traj] = multiUAVs(UAVnum, worldin)
start=zeros(UAVnum,3);
goal=zeros(UAVnum,3);

path=cell(1,UAVnum);
tree=cell(1,UAVnum);
inflec=cell(1,UAVnum);
traj=cell(1,UAVnum);

% generate start & goal point
for i=1:UAVnum
    start(i,1)=10*rand;
    start(i,2)=10*rand;
    start(i,3)=10*rand;
    goal(i,1)=10*rand+90;
    goal(i,2)=10*rand+90;
    goal(i,3)=10*rand+90;
end

for i=1:UAVnum
    [pathTmp,treeTmp]=getPath(start(i,:), goal(i,:), worldin);
    path{i}=pathTmp;tree{i}=treeTmp;
    [inflec{i},~,traj{i}]=getTrajectory(path{i});
    % % draw RRT result
    % figure
    % plotExpandedTree(world,tree{i},3);
    % plotWorld(world,path{i},1);
end

end

% generate multiple UAVs' start and end point randomly in a certain range
% and plan path and calculate trajectory, however error happens when
% applied in main file, no more use.

