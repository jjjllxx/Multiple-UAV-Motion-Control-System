function [] = showTraj(world, trajectory, figure_title)
%SHOWTRAJ Summary of this function goes here
%   Detailed explanation goes here
num=size(trajectory,2);

figure
for ob=1:world.NumObstacles
    [X,Y,Z] = sphere(10);
    X = (X*world.radius(ob));
    Y = (Y*world.radius(ob));
    Z = (Z*world.radius(ob));
    surf(X+world.cx(ob),Y+world.cy(ob),Z+world.cz(ob),'FaceAlpha',0.2);
    colormap([0.5 0.2 0.3]);
    hold on
end
for i=1:num
    now=trajectory{i};
    plot3(now(1,:), now(2,:), now(3,:), 'LineWidth', 2)
    hold on
end
title(figure_title)
axis([0 100, 0 100, 0 100])
% saveas(gcf, [figure_title,'.png'])
end

