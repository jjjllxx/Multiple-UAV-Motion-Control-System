% start_cord1 = [5,5,5];
% goal_cord1 = [95,95,95];
% [path1,tree1]=getPath(start_cord1,goal_cord1,world);
% 
% % find path with minimum cost to end_node
% figure
% plotExpandedTree(world,tree1,dim);
% plotWorld(world,path1,1);

% figure
% % plotExpandedTree(world,tree,dim);
% plotWorld(world,path1,2);






% [flag2,loc1,cp2]=judgeCol(inflec{1}(2:4,:),inflec{2}(2:4,:),colLimit);

% figure
% plot3(inflec1(2,:),inflec1(3,:),inflec1(4,:));
% hold on
% plot3(inflec2(2,:),inflec2(3,:),inflec2(4,:));
% legend('path1','path2')
% 
% inflec1=inflec1(2:4,:);inflec2=inflec2(2:4,:);
% UAVsize=1;colLimit=3*UAVsize;
% [flag2,loc1,cp2]=judgeCol(inflec1,inflec2,colLimit);
% 
% test1=showDistance(inflec1(:,loc1:loc1+1)',inflec2(:,flag2:flag2+1)');
% 
% if flag2
%     new2=replanPath(inflec1,inflec2,cp2,flag2);
% end
% [flag2new,~,cp2new]=judgeCol(inflec1,new2,colLimit);
% test2=showDistance(inflec1(:,loc1:loc1+1)',new2(:,flag2:flag2+1)');