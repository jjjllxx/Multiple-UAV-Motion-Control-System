function [] = showOldNew(traj1,traj2,number)

t1=1:size(traj1,2);
t2=1:size(traj2,2);
figure
subplot(311)
plot(t1,traj1(1,:))
hold on
plot(t2,traj2(1,:))
legend('oldTraj-x','newTraj-x','Location','Best')
title(['Comparison of old and new trajectory',num2str(number),'(x-axis)'])
xlabel('time/s') 
ylabel('x-location') 

subplot(312)
plot(t1,traj1(2,:))
hold on
plot(t2,traj2(2,:))
legend('oldTraj-y','newTraj-y','Location','Best')
title(['Comparison of old and new trajectory',num2str(number),'(y-axis)'])
xlabel('time/s') 
ylabel('y-location') 



subplot(313)
plot(t1,traj1(3,:))
hold on
plot(t2,traj2(3,:))
legend('oldTraj-z','newTraj-z','Location','Best')
title(['Comparison of old and new trajectory',num2str(number),'(y-axis)'])
xlabel('time/s') 
ylabel('z-location') 


end

% this function is used as comparison of new and old trajectory
