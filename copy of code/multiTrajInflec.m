function [] = multiTrajInflec(toDraw)
flag=size(toDraw{1},1);
n=size(toDraw,2);
if flag==3
    figure
    for p=1:3
        h=cell(1,n);
        for i=1:n
            nowTraj=toDraw{i};
            t=1:size(nowTraj,2);
            subplot(3,1,p)
            plot(t,nowTraj(p,:));
            hold on
            h{i}=['traj', num2str(i),'-',char(119+p)];
        end
        legend(h,'Location','Best')
        xlabel('time/s') 
        ylabel([char(119+p),'-location']) 
        title(['time-based trajectory(',char(119+p),'-axis)'])
    end 
end

if flag==4
    figure
    for i=1:n
        nowInflec=toDraw{i};
        plot3(nowInflec(2,:),nowInflec(3,:),nowInflec(4,:));
        hold on
        h{i}=['Path',num2str(i)];
    end
    legend(h,'Location','Best')
    title('RRT path extraction')
end
end

% draw trajectory and inflection(keypoint) for multiple UAVs,
% trajectory/inflection are packed in cell, no matter the number of it.
