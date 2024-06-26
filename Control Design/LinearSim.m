function [t,x,u,duty] = LinearSim(tspan,xe,x0,A,B,K,p)
dx0 = x0 - xe; 

[t,dx]= ode45(@(t,dx)linearStatespace(A,t,dx,B,K),tspan,dx0);

dx = dx';
x = zeros(12,length(t));
u = zeros(4,length(t));
duty = zeros(4,length(t));
for i = 1:length(t)
    u(:,i) = -K*dx(:,i);
    x(:,i) = xe + dx(:,i);
    duty(:,i) = ((u(:,i)+p.rho*p.pi).^2 + 11.1*p.rho - (p.rho*p.pi)^2)./(222*p.rho);
end

end