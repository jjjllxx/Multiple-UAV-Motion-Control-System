function [tdis]=showDistance(seg1,seg2)
UAVsize=1;colLimit=3*UAVsize;

x0=seg2(1,1);x1=seg2(2,1);
y0=seg2(1,2);y1=seg2(2,2); 
z0=seg2(1,3);z1=seg2(2,3); 

q1=seg1(1,:);q2=seg1(2,:);
vector1=q2-q1;vector2=q1-q2;
tdis=zeros(1,101);
t=0:0.01:1;
xdis=x0+t*(x1-x0);ydis=y0+t*(y1-y0);zdis=z0+t*(z1-z0);
for i=1:101
    pt=[xdis(i),ydis(i),zdis(i)];
    if (vector1.*(pt-q1)<=0) | (vector2.*(pt-q1)<=0)
        d=min(norm(pt-q1),norm(pt-q2));
    else
        d=norm(cross(pt-q1,pt-q2))/norm(q1-q2);
    end
    tdis(i)=d;
end
dmax=max(tdis)*1.3;
% draw distance
figure
subplot(311)
plot(xdis,tdis)
hold on
plot(xdis,colLimit*ones(1,101))
legend('x','threshold')
xmin=min(xdis);xmax=max(xdis);
axis([xmin,xmax,0,dmax])

subplot(312)
plot(ydis,tdis)
hold on
plot(ydis,colLimit*ones(1,101))
legend('y','threshold')
ymin=min(ydis);ymax=max(ydis);
axis([ymin,ymax,0,dmax])

subplot(313)
plot(zdis,tdis)
hold on
plot(zdis,colLimit*ones(1,101))
legend('z','threshold')
zmin=min(zdis);zmax=max(zdis);
axis([zmin,zmax,0,dmax])

end

% for 2 segments, this function show the distance between then

