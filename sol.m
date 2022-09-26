function [d,x,y] = sol(k,b,x_real,y_real) 
    d = abs(k*x_real + b - y_real)/sqrt(1+k^2);
    sign = (k*x_real + b - y_real)/abs(k*x_real + b - y_real);
    x = x_real + sign*k/sqrt(1+k^2)*d;
    y = y_real + sign*k/sqrt(1+k^2)*d;
    end