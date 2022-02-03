clear;close;clc;
p1=[38,5,3];p2=[18,18,28];q1=[40,5,5];q2=[20,20,30];
s1=p2-p1;s2=q2-q1;
res1=((s1*s2')*((p1-q1)*s2')-(s2*s2')*((p1-q1)*s1'))/((s1*s1')*(s2*s2')-(s1*s2')*(s1*s2'));     %lamta1
res2=-((s1*s2')*((p1-q1)*s1')-(s1*s1')*((p1-q1)*s2'))/((s1*s1')*(s2*s2')-(s1*s2')*(s1*s2'));    %lamta2
if(res1<=1&&res1>=0&&res2<=1&&res2>=0)  %如果两个垂足都在两个直线段上，则将lambda带入，可得到两个垂足坐标，求其长度即可
    tmp1=p1+res1*s1;
    tmp2=q1+res2*s2;
    tmp=tmp1-tmp2;
    distance=sqrt(tmp*tmp');
else
    res3=(q1-p1)*s1'/(s1*s1');           %如果有垂足不落在线段上，需要进行下一步判断，分别计算两个线段的四个坐标点，到另外一条线段的长度
    if (res3>=0&&res3<=1)                %q1点到p1p2线段的距离，同样计算lamta,如果在0~1表示垂足在线段上，带入求值
         tmp=q1-(p1+res3*s1);
         d1=sqrt(tmp*tmp');
    else
         d1=sqrt(min((q1-p1)*(q1-p1)',(q1-p2)*(q1-p2)'));%如果垂足不在线段上，直接判断到达两个端点的线段长度即可，取最小
    end
    res4=(q2-p1)*s1'/(s1*s1');           %q2到线段p1p2距离
    if (res4>=0&&res4<=1)
        tmp=q2-(p1+res4*s1);
        d2=sqrt(tmp*tmp');
    else
        d2=sqrt(min((q2-p1)*(q2-p1)',(q2-p2)*(q2-p2)'));
    end

    res5=(p1-q1)*s2'/(s2*s2');            %p1到线段q1q2距离
   if (res5>=0&&res5<=1)
        tmp=p1-(q1+res5*s2);
        d3=sqrt(tmp*tmp');
   else
        d3=sqrt(min((p1-q1)*(p1-q1)',(p1-q2)*(p1-q2)'));
   end

   res6=(p2-q1)*s2'/(s2*s2');            %p2到线段q1q2距离
   if (res6>=0&&res6<=1)
       tmp=p2-(q1+res6*s2);
       d4=sqrt(tmp*tmp');
   else
       d4=sqrt(min((p2-q1)*(p2-q1)',(p2-q2)*(p2-q2)'));
   end
   distance=min(min(d1,d2),min(d3,d4));%取四个最短的一个即可
end
