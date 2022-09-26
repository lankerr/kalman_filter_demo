clc;clear;
errsum = zeros(1,10);
angle_gen_v2
rohvar = 367.5;
thetavar = 0.2875;
angle_gen
%我们在这里仿真1,2的东西
opttime = 10;
runtime = 1000;
for j = 1:runtime
%接收机的编号
d = randi([3,9]);
err = zeros(1,opttime);
roh = normrnd(100,sqrt(rohvar));
theta_r = 40*(d-1) + normrnd(0,sqrt(thetavar));
theta = theta_r/180*pi;
x(1) = roh * cos(theta);
y(1) = roh * sin(theta);
for i = 1:opttime
    [x1,y1,err(i)] = kalmanopt(1,2,d,x(i),y(i));
    x(i+1) = x1;
    y(i+1) = y1;
end
errsum = errsum + err;
end
plot(errsum/10000);
grid on
title('带噪声的（100dB的误差）')
xlabel('优化次数');
ylabel('平均误差/m');