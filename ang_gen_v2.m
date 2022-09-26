function [alpha1,alpha2,alpha3] = ang_gen_v2(i,j,k,x1_k,y1_k)
%i,j是两个，发射器
angle_gen_v2%我们现在把x,y当作一个全局的变量来看，将会变得重要

x1 = x(i);
x2 = x(j);
x3 = x1_k;
y1 = y(i);
y2 = y(j);
y3 = y1_k;

l1 = [x3,y3];
l2 = [x3-x1,y3-y1];
l3 = [x3-x2,y3-y2];

d1 = abs(i-k);
d2 = abs(j-k);
d3 = abs(i-j);
if(min(d1,9-d1)+min(d2,9-d2) == min(d3,9-d3))
    si = -1;
else 
    si = 1;
end

alpha_1 = acos(abs(dot(l1,l2))/(norm(l1)*norm(l2)));
alpha_2 = acos(si*abs(dot(l2,l3))/(norm(l2)*norm(l3)));
alpha_3 = acos(abs(dot(l1,l3))/(norm(l1)*norm(l3)));

t = [alpha_1,alpha_2,alpha_3];
t = sort(t);

alpha1 = t(1);
alpha2 = t(2);
alpha3 = t(3);

end