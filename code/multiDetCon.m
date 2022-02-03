function [traj_new,collison_time] = multiDetCon(traj_all, inflec_all, colLimit)
multiTrajInflec(inflec_all);
multiTrajInflec(traj_all);
n=size(traj_all,2);
traj_new=cell(1,n);collison_time=cell(1,n-1);
traj_new{1}=traj_all{1};
for t=1:n-1
    [~,coltime]=timeCol(traj_new{t},traj_all{t+1},colLimit);
    collison_time{t}=coltime;
    if size(coltime,2)>1
        [~,newTraj]=slowTrajectory(inflec_all{t+1},coltime);
        showOldNew(traj_all{t+1},newTraj,t+1);
        traj_new{t+1}=newTraj;
    else
        traj_new{t+1}=traj_all{t+1};
    end
end
multiTrajInflec(traj_new);
end

% collision detection for every trajectory in the cell