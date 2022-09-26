function [theT,theR] = location(i,j,a1,a2,a3,k,R) 
% 圆心无人机和圆周上编号为i和j的无人机发射信号，有i<j
%被测无人机编号k，得到a1，a2为小角，a3为大角
% R为圆形编队的半径
gap=min(j-i-1,i-j+8);

if j-i-1==gap
    store=(i-1)*40/180*pi; % 顺时针转
    k=mod(k-i+1+9,9);
else
    store=-(10-j)*40/180*pi;% 逆时针转
    k=mod(k+(10-j),9);
end
j=gap+2; i=1;

t1=(i-1)*40/180*pi;
t2=(j-1)*40/180*pi;
rightR=R;   rightT=(k-1)*40/180*pi;
switch gap
    case 0
        if k==mod(j+4,9)  % 圆心连线当公共边，两个小角画圆
            % 首先a1和t1配对，得到解
            [theT,theR]=anticlock(a2,t2,a1,t1,R);
            % a1和t2配对
            [tempT,tempR]=anticlock(a1,t2,a2,t1,R);
        elseif k<mod(4+j,9) && k>=mod(j+1,9) %大角和其中一个小角画圆
            % a3和t2（j）配对
            % a1和t1配对
            [theT,theR]=antianti(a1,t1,a3,t2,R);
            % a2和t1配对
            [tempT,tempR]=antianti(a2,t1,a3,t2,R);
        else
            % a3和t1（i）配对
            % a1和t2配对
            [theT,theR]=clockclock(a3,t1,a1,t2,R);
            % a2和t2配对
            [tempT,tempR]=clockclock(a3,t1,a2,t2,R);
        end
    case 1
        if k==mod(j+3,9) || k==mod(j+4,9)  % 圆心连线当公共边，两个小角画圆
            [theT,theR]=anticlock(a1,t2,a2,t1,R);
            [tempT,tempR]=anticlock(a2,t2,a1,t1,R);
        elseif k==j-1
            [theT,theR]=anticlock(a1,t1,a2,t2,R);
            [tempT,tempR]=anticlock(a2,t1,a1,t2,R);
        elseif k<mod(j+3,9) && k>=mod(j+1,9) %大角和其中一个小角画圆
            % a3和t2（j）配对
            % a1和t1配对
            [theT,theR]=antianti(a1,t1,a3,t2,R);
            % a2和t1配对
            [tempT,tempR]=antianti(a2,t1,a3,t2,R);
        else
            % a3和t1（i）配对
            % a1和t2配对
            [theT,theR]=clockclock(a3,t1,a1,t2,R);
            % a2和t2配对
            [tempT,tempR]=clockclock(a3,t1,a2,t2,R);
        end        
    case 2
        if  k==mod(j+2,9) || k==mod(j+3,9) || k==mod(j+4,9)  % 圆心连线当公共边，两个小角画圆
            [theT,theR]=anticlock(a1,t2,a2,t1,R);
            [tempT,tempR]=anticlock(a2,t2,a1,t1,R);
        elseif k<j && k>i
            [theT,theR]=anticlock(a1,t1,a2,t2,R);
            [tempT,tempR]=anticlock(a2,t1,a1,t2,R);
        elseif k==j+1 %大角和其中一个小角画圆
            % a3和t2（j）配对
            % a1和t1配对
            [theT,theR]=antianti(a1,t1,a3,t2,R);
            % a2和t1配对
            [tempT,tempR]=antianti(a2,t1,a3,t2,R);
        else
            % a3和t1（i）配对
            % a1和t2配对
            [theT,theR]=clockclock(a3,t1,a1,t2,R);
            % a2和t2配对
            [tempT,tempR]=clockclock(a3,t1,a2,t2,R);
        end        
    case 3
        if k<j && k>i %大角和其中一个小角画圆
           [theT,theR]=anticlock(a1,t1,a2,t2,R);
           [tempT,tempR]=anticlock(a2,t1,a1,t2,R);
        else
           [theT,theR]=anticlock(a1,t2,a2,t1,R);
           [tempT,tempR]=anticlock(a2,t2,a1,t1,R);
        end        
end


if dist(theR,rightR,theT,rightT)>dist(tempR,rightR,tempT,rightT)
    theT=tempT;
    theR=tempR;
end

theT=theT+store; %转回来
% 
% if theR<0
%     theR=-theR;
%     theT=theT-pi;
% end
% 
% if theT<0
%     theT=theT+2*pi;
% end

end

