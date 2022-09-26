clc;clear;
rohvar = 36.75;
thetavar = 0.02875;
simtime = 1000;
angle_gen
%我们在这里仿真1,2的东西
roh = zeros(1,simtime);
theta_r = zeros(1,simtime);
theta = zeros(1,simtime);
err=zeros(1,simtime);

%接收机的编号
d = randi([3,9],1,simtime);
for i = 1:simtime
%    if(d(i) ==1||d(i)==0)
%        err(i) = 0;
%        continue;
%    end
   roh(i) = normrnd(100,sqrt(rohvar));
   theta_r(i) = 40*(d(i)-1) + normrnd(0,sqrt(thetavar));
   theta(i) = theta_r(i)/180*pi;
   [alpha1(i),alpha2(i),alpha3(i)] = ang_gen(1,2,d(i),roh(i),theta(i));
   [m(i),n(i)] = location(1,2,alpha1(i),alpha2(i),alpha3(i),d(i),100);
   %err(i) = dist(n(i),roh(i),m(i),theta(i));
   err(i) = dist(n(i),100,m(i),40*(d(i)-1)*pi/180);
end

errsum = sum(err>10);
errate = errsum/simtime;