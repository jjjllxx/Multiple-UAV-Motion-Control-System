clc;clear;close all;
dim=3;

% create world
random_world = 0;
Size = 100;
NumObstacles = 100;

if random_world ==1
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim));
else
    [world,NumObstacles] = createKnownWorld(ones(1,dim)*Size,[0;0;0]);
end

start_cord1 = [5,5,5];
goal_cord1 = [95,95,95];
path1=basic(start_cord1,goal_cord1,world,[]);

start_cord2 = [5,5,5];
goal_cord2 = [95,95,95];
path2=basic(start_cord2,goal_cord2,world,path1);

start_cord3 = [95,95,5];
goal_cord3 = [5,5,95];
path3=basic(start_cord3,goal_cord3,world,[path1;path2]);

figure
% plotExpandedTree(world,tree);
plotWorld(world,path1,2);
hold on
plotWorld(world,path2,2);
plotWorld(world,path3,2);

drawTrajectory(path1,path2,path3)

function path1 = basic(start_cord,goal_cord,world,existed_path)
segmentLength=5;

start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
% establish tree starting with the start node
tree = start_node;

numPaths = 0;
flag = 0;
while numPaths<1
    [tree,flag] = extendTree(tree,end_node,segmentLength,world,existed_path);
    numPaths = numPaths + flag;
end

path1 = findMinimumPath(tree,end_node);


% find path with minimum cost to end_node
figure
plotExpandedTree(world,tree);
plotWorld(world,path1,1);
end

function world = createWorld(NumObstacles, endcorner, origincorner)
% check to make sure that the region is nonempty
if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
    disp('Not valid corner specifications!')
    world=[];
      
else
    % create world data structure
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


function [world,NumObstacles] = createKnownWorld(endcorner, origincorner)
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
    maxRadius = 10;
    
    world.radius(1) = maxRadius;
    cx = 50;
    cy = 50;
    cz = 50;
    world.cx(1) = cx;
    world.cy(1) = cy;
    world.cz(1) = cz;
    
    world.radius(2) = maxRadius;
    cx = 25;
    cy = 25;
    cz = 25;
    world.cx(2) = cx;
    world.cy(2) = cy;
    world.cz(2) = cz;
    
    world.radius(3) = maxRadius;
    cx = 75;
    cy = 75;
    cz = 75;
    world.cx(3) = cx;
    world.cy(3) = cy;
    world.cz(3) = cz;
    
    world.radius(4) = maxRadius;
    cx = 25;
    cy = 25;
    cz = 75;
    world.cx(4) = cx;
    world.cy(4) = cy;
    world.cz(4) = cz;
    
    world.radius(5) = maxRadius;
    cx = 75;
    cy = 75;
    cz = 25;
    world.cx(5) = cx;
    world.cy(5) = cy;
    world.cz(5) = cz;
    
    world.radius(6) = maxRadius;
    cx = 25;
    cy = 75;
    cz = 25;
    world.cx(6) = cx;
    world.cy(6) = cy;
    world.cz(6) = cz;
    
    world.radius(7) = maxRadius;
    cx = 75;
    cy = 25;
    cz = 25;
    world.cx(7) = cx;
    world.cy(7) = cy;
    world.cz(7) = cz;
    
    world.radius(8) = maxRadius;
    cx = 75;
    cy = 25;
    cz = 75;
    world.cx(8) = cx;
    world.cy(8) = cy;
    world.cz(8) = cz;
    
    
    world.radius(9) = maxRadius;
    cx = 25;
    cy = 75;
    cz = 75;
    world.cx(9) = cx;
    world.cy(9) = cy;
    world.cz(9) = cz;
end
end


function collision_flag = collision(node,parent,world,path)
dim=3;
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
threshold=2;
if collision_flag == 0 && size(path,1)~=0
    path_length=size(path,1);
    for sigma = 0:.2:1
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        
        for i=2:path_length-1
            if (norm([p(1);p(2);p(3)]-[path(i,1); path(i,2); path(i,3)]))<=threshold
                collision_flag = 1;
                break;
            end
        end
    end
end

end



function collision_flag = is_point_valid(point, world,path)
dim=3;
collision_flag = 0;

for i=1:dim
    if (point(i)>world.endcorner(i))||(point(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0
    p = point(1:dim);
    % check each obstacle
    for i=1:world.NumObstacles
        if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i))
            collision_flag = 1;
            break;
        end
    end
end

threshold=2;
if collision_flag == 0 && size(path,1)~=0
    path_length=size(path,1);
    for sigma = 0:.2:1
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        
        for i=2:path_length-1
            if (norm([p(1);p(2);p(3)]-[path(i,1); path(i,2); path(i,3)]))<=threshold
                collision_flag = 1;
                break;
            end
        end
    end
end
end



function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world,path)
dim=3;
flag = 0;
% select a random point
randomPoint = zeros(1,dim);
for i=1:dim
    randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
end

% find leaf on node that is closest to randomPoint
tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
sqrd_dist = sqr_eucl_dist(tmp);
[~,idx] = min(sqrd_dist);

min_parent_idx = idx;

new_point = tree(idx,1:dim);

pflag = 0;


while norm(new_point-randomPoint)>0 && pflag==0
    
    
    if norm(new_point-randomPoint)<segmentLength
        
        pflag = collision(randomPoint,tree(min_parent_idx,:),world,path);
        if pflag == 0
            
            new_point = randomPoint;
            min_cost = cost_np(tree(min_parent_idx,:),new_point);
            new_node = [new_point,0,min_cost,min_parent_idx];
            tree = [tree;new_node];
            pflag = 1;
            goal_flag = is_goal(new_node,end_node,segmentLength,world,path);
            
            if goal_flag == 1
                tree(end,dim+1)=1;
                flag = 1;
            end
        end
        
    else
        
        new_point = (randomPoint-tree(min_parent_idx,1:dim));
        new_point = tree(min_parent_idx,1:dim)+(new_point/norm(new_point))*segmentLength;
        
        min_cost  = cost_np(tree(min_parent_idx,:),new_point);
        new_node  = [new_point, 0, min_cost, min_parent_idx];
        
        pflag = collision(new_node,tree(min_parent_idx,:),world,path);
        
        if pflag == 0
            tree = [tree ; new_node];
            min_parent_idx = size(tree,1);
            
            goal_flag = is_goal(new_node,end_node,segmentLength,world,path);
            
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



function goal_flag = is_goal(node,end_node,segmentLength,world,path)
dim=3;
goal_flag = 0;
if (norm(node(1:dim)-end_node(1:dim))<segmentLength )...
        && (collision(node,end_node,world,path)==0)
    goal_flag = 1;
end

end



function e_dist = sqr_eucl_dist(array)
dim=3;
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
function [cost] = cost_np(from_node,to_point)
dim=3;
diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end


function path = findMinimumPath(tree,end_node)
dim=3;
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


function plotExpandedTree(~,tree)
dim=3;
ind = size(tree,1);
while ind>0
    size(tree);
    branch = [];
    node = tree(ind,:);
    branch = [ branch ; node ];
    parent_node = node(dim+3);
    while parent_node > 1
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
