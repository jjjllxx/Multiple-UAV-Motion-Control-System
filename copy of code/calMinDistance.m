function [distance,point] = calMinDistance(p1,p2,q1,q2)
s1=p2-p1;s2=q2-q1;
res1=((s1*s2')*((p1-q1)*s2')-(s2*s2')*((p1-q1)*s1'))/((s1*s1')*(s2*s2')-(s1*s2')*(s1*s2'));     %lambda1
res2=-((s1*s2')*((p1-q1)*s1')-(s1*s1')*((p1-q1)*s2'))/((s1*s1')*(s2*s2')-(s1*s2')*(s1*s2'));    %lambda2
if(res1<=1&&res1>=0&&res2<=1&&res2>=0)  %if lambda1 and lambda2 are in range of (0,1),the minDistance can be calculated directly
    tmp1=p1+res1*s1;
    tmp2=q1+res2*s2;
    tmp=tmp1-tmp2;
    distance=sqrt(tmp*tmp');
    point=tmp2;
else
    res3=(q1-p1)*s1'/(s1*s1');           %otherwise
    if (res3>=0&&res3<=1)                %q1点到p1p2线段的距离，lambda,如果在0~1表示垂足在线段上，带入求值
        tmp=q1-(p1+res3*s1);
        d1=sqrt(tmp*tmp');
    else
        d1=sqrt(min((q1-p1)*(q1-p1)',(q1-p2)*(q1-p2)'));%if the foot of pendicular not on another line,take the smaller of q1 to p1 and p2
        
    end
    point1=q1;
    
    res4=(q2-p1)*s1'/(s1*s1');           %q2 to p1p2
    if (res4>=0&&res4<=1)
        tmp=q2-(p1+res4*s1);
        d2=sqrt(tmp*tmp');
    else
        d2=sqrt(min((q2-p1)*(q2-p1)',(q2-p2)*(q2-p2)'));
    end
    point2=q2;
    
    res5=(p1-q1)*s2'/(s2*s2');            %p1 to q1q2
    if (res5>=0&&res5<=1)
        tmp=p1-(q1+res5*s2);
        d3=sqrt(tmp*tmp');
        point3=q1+res5*s2;
    else
        d3=sqrt(min((p1-q1)*(p1-q1)',(p1-q2)*(p1-q2)'));
        if d3==(p1-q1)*(p1-q1)'
            point3=q1;
        else
            point3=q2;
        end
    end
    
    res6=(p2-q1)*s2'/(s2*s2');            %p2 to q1q2
    if (res6>=0&&res6<=1)
        tmp=p2-(q1+res6*s2);
        d4=sqrt(tmp*tmp');
        point4=q1+res6*s2;
    else
        d4=sqrt(min((p2-q1)*(p2-q1)',(p2-q2)*(p2-q2)'));
        if d3==(p2-q1)*(p2-q1)'
            point4=q1;
        else
            point4=q2;
        end
    end
    distance=min(min(d1,d2),min(d3,d4));%minDistance is the minimum of four
    if distance==d1
        point=point1;
    elseif distance==d2
        point=point2;
    elseif distance==d3
        point=point3;
    else
        point=point4;
    end     
end

end

% calculate the min distance of two segment
