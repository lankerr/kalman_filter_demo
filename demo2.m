rohvar = 37.7197;
thetavar = 0.0074;
simtime = 100000;
num = 10;
angle_gen
%我们在这里仿真1 r(i)的东西
roh = zeros(1,simtime);
theta_r = zeros(1,simtime);
theta = zeros(1,simtime);
d = zeros(1,simtime);
r = zeros(1,simtime);
%d为另外一个发射机，r为被动接受的机器
for i = 1:simtime
   s = randsample(2:9,2);
   d(i) = s(1);
   r(i) = s(2);
   roh(i) = normrnd(100,sqrt(rohvar));
   theta_r(i) = 40*(r(i)-1) + normrnd(0,sqrt(thetavar));
   theta(i) = theta_r(i)/360*2*pi;
   [alpha1(i),alpha2(i),alpha3(i)] = ang_gen(1,d(i),r(i),roh(i),theta(i));
   [m(i),n(i)] = location(1,d(i),alpha1(i),alpha2(i),alpha3(i),r(i),100);
   err(i) = dist(n(i),100,m(i),40*(r(i)-1)*pi/180);
end

errsum = sum(err>10);
errate = errsum/(simtime*num);