function D= dist(rho1,rho2,theta1,theta2)
D=sqrt(rho1^2+rho2^2-2*rho1*rho2*cos(theta1-theta2));
end

