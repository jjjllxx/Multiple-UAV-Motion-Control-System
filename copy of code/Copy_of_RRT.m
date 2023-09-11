clc;clear;close all;
dim = 3;
random_world = 0;

% create random world
world_size = 100;
NumObstacles = 100;
if random_world == 1
    world = createWorld(NumObstacles,ones(1, dim)*world_size, zeros(1, dim));
else
    [world, NumObstacles] = createKnownWorld(ones(1, dim) * world_size,[0; 0; 0], world_size);
end


UAV_num = 2; % number of UAVs
start = zeros(UAV_num, 3); 
goal = zeros(UAV_num, 3);
path = cell(1, UAV_num);
tree = cell(1, UAV_num);
inflec = cell(1, UAV_num);
traj = cell(1, UAV_num);

% generate start & goal point
for i=1:UAV_num
    start(i, 1) = i * rand;  
    goal(i, 1) = i * rand + (world_size - UAV_num);
    start(i, 2) = i * rand; 
    goal(i, 2) = i * rand + (world_size - UAV_num);
    start(i, 3) = i * rand;  
    goal(i, 3) = i * rand + (world_size - UAV_num);
end
% generate RRT path

for i=1:UAV_num
    [path{i}, tree{i}] = getPath(start(i, :), goal(i, :), world);
    [~, ~, traj{i}] = getTrajectory(path{i});
%     % draw RRT result
%     figure
%     plotExpandedTree(world, tree{i}, 3);
%     plotWorld(world, path{i},1);
end

figure
for i = 1 : UAV_num
    plotWorld(world, path{i}, 2);
    hold on
end
saveas(gcf, 'RRT Path.png')

traj0=cell(1,UAV_num);
traj1=cell(1,UAV_num);
traj2=cell(1,UAV_num);
% different velocity at keypoints

for v = [0, 1, 2, 3]
    for i = 1:UAV_num
        [~, ~, traj0{i}] = getTraNew(path{i}, v);
        [~, ~, traj1{i}] = getTraMode(path{i}, v);
        [~, ~, traj2{i}] = getTraBS(path{i}, v);
    end  
    showTraj(world, traj0, ['trajectory(keypoint velocity=', num2str(v), ')'])
    showTraj(world, traj1, ['trajectory(recovery)(keypoint velocity=', num2str(v), ')'])
    showTraj(world, traj2, ['trajectory(approach)(keypoint velocity=', num2str(v), ')'])
end



% UAVsize=1;colLimit=3*UAVsize;
% [newTrajectory, coltime]=multiDetCon(traj,inflec,colLimit);
% 
% getGIFTra(world, newTrajectory, -37.5, 30);
% getGIFTra(world, newTrajectory, -120, 20);
% getGIFTra(world, newTrajectory, -15, 30);
% getGIFTra(world, newTrajectory, 80, 20);
% getGIFTra(world, newTrajectory, 0, 90);
% 

% twoTrajDis(newTrajectory{1},newTrajectory{2},colLimit);

% multiTrajInflec(traj);
% multiTrajInflec(inflec);
% 
% UAVsize=1;colLimit=3*UAVsize;
% [colnum,coltime]=timeCol(traj{1},traj{2},colLimit);
% [~,newTraj]=slowTrajectory(inflec{2},coltime);
% showOldNew(traj{2},newTraj,2);
% 
% twoTrajDis(traj{1},newTraj);
% [newcolnum, newcoltime]=timeCol(traj{1},newTraj,colLimit);




function [path,tree]=getPath(start_cord, goal_cord, world)
% standard length of path segments
dim=3;segmentLength = 5;
start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];

% establish tree starting with the start node
tree = start_node;


% check to see if start_node connects directly to end_node
if ((norm(start_node(1:dim)-end_node(1:dim))<segmentLength)&&(collision(start_node,end_node,world,dim)==0))
    path = [start_node; end_node];
else
    nIterations = 0;
    numPaths = 0;
    flag = 0;
    while numPaths<1
        [tree,flag] = extendTree(tree,end_node,segmentLength,world,dim);
        numPaths = numPaths + flag;
        nIterations = nIterations+1;
    end
end


path = findMinimumPath(tree,end_node,dim);
end


function world = createWorld(NumObstacles, endcorner, origincorner)
% check to make sure that the region is nonempty
if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
    disp('Not valid corner specifications!')
    world=[];
    
    % create world data structure
else
    world.NumObstacles = NumObstacles;
    world.endcorner = endcorner;
    world.origincorner = origincorner;
    
    % create NumObstacles
    bounds = [endcorner(1)- origincorner(1), endcorner(2)-origincorner(2), endcorner(3)-origincorner(3)];
    maxRadius = min(bounds);
    maxRadius = 5*maxRadius/NumObstacles;
    for i=1:NumObstacles
        % randomly pick radius
        world.radius(i) = maxRadius*rand;
        % randomly pick center of obstacles
        cx = origincorner(1) + world.radius(i)...
            + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
        cy = origincorner(2) + world.radius(i)...
            + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
        cz = origincorner(2) + world.radius(i)...
            + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
        world.cx(i) = cx;
        world.cy(i) = cy;
        world.cz(i) = cz;
    end
end
end



function [world,NumObstacles] = createKnownWorld(endcorner, origincorner, world_size)
NumObstacles = 9;
% check to make sure that the region is nonempty
if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
    disp('Not valid corner specifications!')
    world=[];
    
    % create world data structure
else
    world.NumObstacles = NumObstacles;
    world.endcorner = endcorner;
    world.origincorner = origincorner;
    
    % create NumObstacles
    maxRadius = 15;
    
    world.radius(1) = maxRadius;
    cx = world_size/2;
    cy = world_size/2;
    cz = world_size/2;
    world.cx(1) = cx;
    world.cy(1) = cy;
    world.cz(1) = cz;
    
    world.radius(2) = maxRadius;
    cx = world_size/4;
    cy = world_size/4;
    cz = world_size/4;
    world.cx(2) = cx;
    world.cy(2) = cy;
    world.cz(2) = cz;
    
    world.radius(3) = maxRadius;
    cx = 0.75*world_size;
    cy = 0.75*world_size;
    cz = 0.75*world_size;
    world.cx(3) = cx;
    world.cy(3) = cy;
    world.cz(3) = cz;
    
    world.radius(4) = maxRadius;
    cx = world_size/4;
    cy = world_size/4;
    cz = 0.75*world_size;
    world.cx(4) = cx;
    world.cy(4) = cy;
    world.cz(4) = cz;
    
    world.radius(5) = maxRadius;
    cx = 0.75*world_size;
    cy = 0.75*world_size;
    cz = world_size/4;
    world.cx(5) = cx;
    world.cy(5) = cy;
    world.cz(5) = cz;
    
    world.radius(6) = maxRadius;
    cx = world_size/4;
    cy = 0.75*world_size;
    cz = world_size/4;
    world.cx(6) = cx;
    world.cy(6) = cy;
    world.cz(6) = cz;
    
    world.radius(7) = maxRadius;
    cx = 0.75*world_size;
    cy = world_size/4;
    cz = world_size/4;
    world.cx(7) = cx;
    world.cy(7) = cy;
    world.cz(7) = cz;
    
    world.radius(8) = maxRadius;
    cx = 0.75*world_size;
    cy = world_size/4;
    cz = 0.75*world_size;
    world.cx(8) = cx;
    world.cy(8) = cy;
    world.cz(8) = cz;
    
    
    world.radius(9) = maxRadius;
    cx = world_size/4;
    cy = 0.75*world_size;
    cz = 0.75*world_size;
    world.cx(9) = cx;
    world.cy(9) = cy;
    world.cz(9) = cz;
end
end



function node=generateRandomNode(world)
% randomly pick configuration
px       = (world.endcorner(1)-world.origincorner(1))*rand;
py       = (world.endcorner(2)-world.origincorner(2))*rand;
pz       = (world.endcorner(3)-world.origincorner(3))*rand;

chi      = 0;
cost     = 0;
node     = [px, py, pz, chi, cost, 0];

% check collision with obstacle
while collision(node, node, world,dim)
    px       = (world.endcorner(1)-world.origincorner(1))*rand;
    py       = (world.endcorner(2)-world.origincorner(2))*rand;
    pz       = (world.endcorner(3)-world.origincorner(3))*rand;
    
    chi      = 0;
    cost     = 0;
    node     = [px, py, pz, chi, cost, 0];
end  
end



function collision_flag = collision(node,parent,world,dim)

collision_flag = 0;
for i=1:dim
    if (node(i)>world.endcorner(i))||(node(i)<world.origincorner(i))
        collision_flag = 1;
    end
end   
if collision_flag == 0
    for sigma = 0:.2:1
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i=1:world.NumObstacles
            if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i))
                collision_flag = 1;
                break;
            end
        end
    end
end
end



function collision_flag = is_point_valid(point, world,dim)

collision_flag = 0;

for i=1:dim
    if (point(i)>world.endcorner(i))||(point(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0 && dim ==3
    p = point(1:dim);
    % check each obstacle
    for i=1:world.NumObstacles
        if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i))
            collision_flag = 1;
            break;
        end
    end
end
end


function flag = canEndConnectToTree(tree,end_node,minDist,world,dim)
flag = 0;
% check only last node added to tree since others have been checked
if ( (norm(tree(end,1:dim)-end_node(1:dim))<minDist)...
        && (collision(tree(end,1:dim), end_node(1:dim), world,dim)==0) )
    flag = 1;
end
end


function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world,dim)

flag = 0;
% select a random point
randomPoint = zeros(1,dim);
for i=1:dim
    randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
end

% find leaf on node that is closest to randomPoint
tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
sqrd_dist = sqr_eucl_dist(tmp,dim);
[~,idx] = min(sqrd_dist);

min_parent_idx = idx;

new_point = tree(idx,1:dim);
new_node = tree(idx,:);

pflag = 0;


while norm(new_point-randomPoint)>0 && pflag==0
    
    
    if norm(new_point-randomPoint)<segmentLength
        
        pflag = collision(randomPoint,tree(min_parent_idx,:),world,dim);
        if pflag == 0
            
            new_point = randomPoint;
            min_cost = cost_np(tree(min_parent_idx,:),new_point,dim);
            new_node = [new_point,0,min_cost,min_parent_idx];
            tree = [tree;new_node];
            pflag = 1;
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            
            if goal_flag == 1
                tree(end,dim+1)=1;
                flag = 1;
            end
        end
        
    else
        
        new_point = (randomPoint-tree(min_parent_idx,1:dim));
        new_point = tree(min_parent_idx,1:dim)+(new_point/norm(new_point))*segmentLength;
        
        min_cost  = cost_np(tree(min_parent_idx,:),new_point,dim);
        new_node  = [new_point, 0, min_cost, min_parent_idx];
        
        pflag = collision(new_node,tree(min_parent_idx,:),world,dim);
        
        if pflag == 0
            tree = [tree ; new_node];
            min_parent_idx = size(tree,1);
            
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            
            if goal_flag == 1
                tree(end,dim+1)=1;  % mark node as connecting to end.
                pflag = 1;
                flag = 1;
            end
        end
    end
end

new_tree = tree;

end



function goal_flag = is_goal(node,end_node,segmentLength,world,dim)

goal_flag = 0;
if (norm(node(1:dim)-end_node(1:dim))<segmentLength )...
        && (collision(node,end_node,world,dim)==0)
    goal_flag = 1;
end

end



function e_dist = sqr_eucl_dist(array,dim)

sqr_e_dist = zeros(size(array,1),dim);
for i=1:dim
    
    sqr_e_dist(:,i) = array(:,i).*array(:,i);
    
end
e_dist = zeros(size(array,1),1);
for i=1:dim
    
    e_dist = e_dist+sqr_e_dist(:,i);
    
end

end



%calculate the cost from a node to a point
function [cost] = cost_np(from_node,to_point,dim)

diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end


%calculate the cost from a node to a node
function [cost] = cost_nn(from_node,to_node,dim)

diff = from_node(:,1:dim) - to_node(:,1:dim);
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end

function [cost] = line_cost(from_node,to_point,dim)
diff = from_node(:,1:dim) - to_point;
cost = norm(diff);
end


function path = findMinimumPath(tree,end_node,dim)

% find nodes that connect to end_node
connectingNodes = [];
for i=1:size(tree,1)
    if tree(i,dim+1)==1
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

% find minimum cost last node
[~,idx] = min(connectingNodes(:,dim+2));

% construct lowest cost path
path = [connectingNodes(idx,:); end_node];
parent_node = connectingNodes(idx,dim+3);
while parent_node>1
    parent_node = tree(parent_node,dim+3);
    path = [tree(parent_node,:); path];
end

end


function plotExpandedTree(~,tree,dim)
ind = size(tree,1);
while ind>0
    size(tree);
    branch = [];
    node = tree(ind,:);
    branch = [ branch ; node ];
    parent_node = node(dim+3);
    while parent_node > 1
        cur_parent = parent_node;
        branch = [branch; tree(parent_node,:)];
        parent_node = tree(parent_node,dim+3);
    end
    ind = ind - 1;
    X = branch(:,1);
    Y = branch(:,2);
    Z = branch(:,3);
    
    p = plot3(X,Y,Z);
    set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
    hold on;
end
end



function plotWorld(world,path,type)
% the first element is the north coordinate
% the second element is the south coordinate
%     axis([world.origincorner(1),world.endcorner(1),...
%         world.origincorner(2), world.endcorner(2),...
%         world.origincorner(3), world.endcorner(3)]);
%     hold on
    
for i=1:world.NumObstacles
    [X,Y,Z] = sphere(10);
    X = (X*world.radius(i));
    Y = (Y*world.radius(i));
    Z = (Z*world.radius(i));
    surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i));
    colormap([0.5 0.2 0.3]);
    hold on
end


X = path(:,1);
Y = path(:,2);
Z = path(:,3);
p = plot3(X,Y,Z);

if type==1
    set(p,'Color','black','LineWidth',3)
elseif type==2
    set(p,'Color','green','LineWidth',1)
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('RRT Connect Algorithm');
end
