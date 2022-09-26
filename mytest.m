%% 
clc;clear;
figure
R=100;
x=ones(9,1)*R; x=[0;x];
y=[0,0:2*pi/9:2*pi-2*pi/9];
ps=polarscatter(y,x,'filled');
ps.SizeData = 200;
ps.MarkerFaceColor = 'red';
ps.MarkerFaceAlpha = .5;
hold on;

%% 参数
a1=30/180*pi;
a2=70/180*pi;
a3=51/180*pi;
t1=0;t2=8*pi/9;
n=1000;

%% 找交点
theta = 0:2*pi/n:2*pi;
rho=ones(1,n+1)*R;
rho1=R/sin(a1)*cos(theta-pi/2+a1-t1);
rho2=R/sin(a2)*cos(theta-pi/2+a2-t2);
%rho3=R/sind(a3)*cosd(rad2deg(theta)-90+a3-40);
polarplot(theta,rho,'black');
hold on;
polarplot(theta,rho1,'g');
hold on;
polarplot(theta,rho2,'b');
hold on

theT=atan(-(sin(a2)*sin(a1-t1)-sin(a1)*sin(a2-t2))/(sin(a2)*cos(a1-t1)-sin(a1)*cos(a2-t2)));
theR=R/sin(a1)*cos(theT-pi/2+a1-t1);
yy=R/sin(a2)*cos(theT-pi/2+a2-t2);

polarscatter(theT,theR,10,'red');