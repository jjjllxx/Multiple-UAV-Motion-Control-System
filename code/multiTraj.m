function [] = multiTraj(varargin)
%MULTITRAJ Summary of this function goes here
%   Detailed explanation goes here
[ax,args,nargs] = axescheck(varargin{:});

figure
for p=1:3
    h=cell(1,nargs);
    for i=1:nargs
        traj=args{i};
        t=1:size(traj,2);
        subplot(3,1,p)
        plot(t,traj(p,:));
        hold on
        h{i}=['trajectory',num2str(i),'-',char(119+p)];
    end
    legend(h,'Location','Best');
    xlabel('time/s') 
    ylabel([char(119+p),'-location']) 
    title(['time-based trajectory(',char(119+p),'-axis)'])
end
end

% draw trajectory of multiple UAVs, the difference from multiTrajInflec
% function is that, input of this one is not packed in cell,but as seperate
% path, no matter the num