%VD arithmetic
clear;
%=====================前提数据准备=========================================
T=1;%1s进行一次数据处理（你也可以理解成一个单位时间）
num=50; 
N=20/T;%我们希望适应性距离为10m
x=zeros(N,1);
y=zeros(N,1);
vx=zeros(N,1);
vy=zeros(N,1);

%===================需要的东西============================================
roh = 103.690;
theta = 4.8850;
the = rand*2*pi;
x(1)=roh*cos(theta);y(1)=roh*sin(theta);%目标的起点
v  = 0.5;%0.5m/s假设无人机
vx(1)=v*cos(the);vy(1)=v*sin(the);%
ax=0;ay=0;%加速度，但是实际上我们用不到
var=1;
%这里我们可以将var设置成我们想要的数据来模仿精度
%=====================运动,匀速直线运动====================================
for i=1:N-1
    ax=0;ay=0;
    vx(i+1)=vx(i);
    vy(i+1)=vy(i);
    x(i+1)=x(i)+vx(i)*T+0.5*ax*T^2;
    y(i+1)=y(i)+vy(i)*T+0.5*ay*T^2;
    %我们在这里需要额外的增加放入仿真系统里面取去生成角度
    [alpha1,alpha2,alpha3] = ang_gen(1,2,8,roh,theta);%这里我们调用的是v2，因为是实际的位置
    [m,n] = location(1,2,alpha1,alpha2,alpha3,8,100);
    x(i) = n*cos(m);
    y(i) = n*sin(m);
end
rex(num,N)=0;
rey(num,N)=0;

%=======================生成真实的运动轨迹=================================

for m=1:num%注意我们这里假设的是我们的
    nx=0;%randn(N,1);
    ny=0;%randn(N,1);
    zx=x+nx;
    zy=y+ny;
    rex(m,1)=2000;
    rey(m,1)=10000;
    ki=0;
    low=1;high=0;
    u=0;ua=0;
    e=0.8;
    xks(1)=zx(1);
    yks(1)=zy(1);
    xks(2)=zx(2);
    yks(2)=zy(2);
    o=[1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];
    h=[1,0,0,0;0,0,1,0];
    g=[T^2/2,0;T,0;0,T^2/2;0,T];
    q=[10000,0;0,10000];
    perr=[var^2,var^2/T,0,0;var*var/T,2*var^2/(T^2),0,0;0,0,var^2,var^2/T;0,0,var^2/T,2*var^2/(T^2)];
    vx=(zx(2)-zx(1))/2;vy=(zy(2)-zy(1))/2;
    xk=[zx(1);vx;zy(1);vy];
    %
    for r=3:N
        if(u<=40)
            if(low==0)
                [o,h,g,q,perr,xk]=lmodeinitial(T,r,zx,zy,vxks,vyks,perr2);
                high=0;
                low=1;
                ua=0;
            end
            z=[zx(r);zy(r)];
            xk1=o*xk; 
            perr1=o*perr*o';
            k1=perr1*h'/(h*perr1*h'+q);
            xk=xk1+k1*(z-h*xk1);
            perr=(eye(4)-k1*h)*perr1;
            xks(r)=xk(1,1);
            yks(r)=xk(3,1);
            vxks(r)=xk(2,1);
            vyks(r)=xk(4,1);
            xk1s(r)=xk1(1,1);
            yk1s(r)=xk1(3,1);
            vxk1s(r)=xk1(2,1);
            vyks1(r)=xk1(4,1);
            perr11(r)=perr(1,1);
            perr12(r)=perr(1,2);
            perr22(r)=perr(2,2);
            if(r>=20)
                v=z-h*xk1;
                w=h*perr*h'+q;
                p=v'/w*v;
                u=e*u+p;
                s(r-19)=u;
            end
        elseif(u>40)
            if(high==0)
                [o,g,h,q,perr,xk]=hmodeinitial(T,r,e,zx,zy,xk1s,yk1s,vxks,vyks,perr11,perr12,perr22);
                high=1;
                low=0;
                for i=r-5:r-1
                    z=[zx(i);zy(i)];
                      xk1=o*xk; 
                      perr1=o*perr*o';
                      k1=perr1*h'/(h*perr1*h'+q);
                      xk=xk1+k1*(z-h*xk1);
                      perr=(eye(6)-k1*h)*perr1;
                      xks(i)=xk(1,1);
                      yks(i)=xk(3,1);
                      vxks(i)=xk(2,1);
                      vyks(i)=xk(4,1);
                      xk1s(i)=xk1(1,1);
                      yk1s(i)=xk1(3,1);
                      vxk1s(i)=xk1(2,1);
                      vyks1(i)=xk1(4,1);
               end
            end
                z=[zx(r);zy(r)];
                      xk1=o*xk; 
                      perr1=o*perr*o';
                      k1=perr1*h'/(h*perr1*h'+q);
                      xk=xk1+k1*(z-h*xk1);
                      perr=(eye(6)-k1*h)*perr1;
                      xks(r)=xk(1,1);
                      yks(r)=xk(3,1);
                      vxks(r)=xk(2,1);
                      vyks(r)=xk(4,1);
                      xk1s(r)=xk1(1,1);
                      yk1s(r)=xk1(3,1);
                      ag=[xk(5,1);xk(6,1)];
                      perr2=perr;
                      ki=ki+1;
                      pm=[perr(5,5),perr(5,6);perr(6,5),perr(6,6)];
                      pa=ag'/(pm)*ag;
                      sa(r)=pa;
         if(ki>5)
             u1=sa(r-4)+sa(r-3)+sa(r-2)+sa(r-1)+sa(r);
             sb(r)=u1;
             if(u1<20)
                 u=0;
             end
         end
        end
     
     rex(m,r)=xks(r);
     rey(m,r)=yks(r);
    end
end
ex=0;ey=0;
eqx=0;eqy=0;
ex1(N,1)=0;ey1(N,1)=0;
qx(N,1)=0;qy(N,1)=0;
for i=1:N
    for j=1:num
        ex=ex+x(i)-rex(j,i);
        ey=ey+y(i)-rey(j,i);
        eqx=eqx+(x(i)-rex(j,i))^2;
        eqy=eqy+(y(i)-rey(j,i))^2;
    end
    ex1(i)=ex/num;
    ey1(i)=ey/num;
    qx(i)=(eqx/num-(ex1(i)^2))^0.5;
    qy(i)=(eqy/num-(ey1(i)^2))^0.5;
    ex=0;eqx=0;ey=0;eqy=0;
end
figure(1);
plot(x,y,'k-',zx,zy,'g:',xks,yks,'r-.');
legend('真实轨迹','观测轨迹','预计轨迹');
%figure(2);
%plot(zx,zy);
%legend('观测图像');
%figure(3)
%plot(xks,yks);legend('估计的图像');
figure(4);plot(qx);
legend('均方差');