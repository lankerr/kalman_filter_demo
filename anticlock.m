function [theT,theR] = anticlock(a1,t1,a2,t2,R)
% a1和t1配对朝逆时针做圆
%  a2和t2配对朝顺时针做圆
theT=atan(-(sin(a2)*sin(a1-t1)-sin(a1)*sin(a2+t2))/(sin(a2)*cos(a1-t1)+sin(a1)*cos(a2+t2)));
theR=R/sin(a1)*cos(theT-pi/2+a1-t1);
end

