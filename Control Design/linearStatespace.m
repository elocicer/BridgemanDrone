function [dxdt,K] = linearStatespace(A,t,x,B,K)
u = -K*x;
dxdt = A*x+B*u;
end
