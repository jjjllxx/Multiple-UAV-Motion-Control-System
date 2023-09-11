a=[15,10];
b=[35,40];
t1=[2,3];
t2=[7,8];
figure
subplot(311)
for i=1:2
    t=t1(i):0.01:t2(i);
    ft=a(i)+(a(i)-b(i))*(t-t1(i)).^2.*(2*(t-t1(i))-3*(t2(i)-t1(i)))/(t2(i)-t1(i))^3;
    plot(t,ft);
    hold on
end

title('original trajectory')
start=max(t1);
final=min(t2);
pos=0;dis=1000000;
for t=start:0.01:final
    ft1=a(1)+(a(1)-b(1))*(t-t1(1)).^2.*(2*(t-t1(1))-3*(t2(1)-t1(1)))/(t2(1)-t1(1))^3;
    ft2=a(2)+(a(2)-b(2))*(t-t1(2)).^2.*(2*(t-t1(2))-3*(t2(2)-t1(2)))/(t2(2)-t1(2))^3;
    if abs(ft1-ft2)<dis
        dis=abs(ft1-ft2);
        pos=t;
    end
end

subplot(312)
t2=[t2(1),t2(2)+pos-t1(2)];
for i=1:2
    t=t1(i):0.01:t2(i);
    ft=a(i)+(a(i)-b(i))*(t-t1(i)).^2.*(2*(t-t1(i))-3*(t2(i)-t1(i)))/(t2(i)-t1(i))^3;
    plot(t,ft);
    hold on
end
title('slow down orange trajectory')
subplot(313)
t2=[t2(1),pos];
for i=1:2
    t=t1(i):0.01:t2(i);
    ft=a(i)+(a(i)-b(i))*(t-t1(i)).^2.*(2*(t-t1(i))-3*(t2(i)-t1(i)))/(t2(i)-t1(i))^3;
    plot(t,ft);
    hold on
end
title('speed up orange trajectory')