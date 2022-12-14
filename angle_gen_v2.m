%angle_gen 生成相应的角度的理想值，本题全局采用角度制
%角度/360*(2*pi) = 弧度
roh = [100 98 112 105 98 112 105 98 112];
theta = [0 40.10 80.21 119.75 159.86 199.96 240.07 280.17 320.28];
for u = 1:9
    x(u) = roh(u)*cos(theta(u)*pi/180);
    y(u) = roh(u)*sin(theta(u)*pi/180);
end

