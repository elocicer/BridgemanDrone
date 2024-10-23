function [dxdt,K] = linearStatespace(A,t,x,B,K)

% Calculate control action
u = -K*x;
% Calculate new states
dxdt = A*x+B*u;
end
