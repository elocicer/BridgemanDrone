function [t,x,u,duty] = LinearSim(tspan,xe,x0,A,B,K,p)

% Linearized initial condition is deviation between initial conditio and
% linearization point
dx0 = x0 - xe; 

% Simulate linearized system
[t,dx]= ode45(@(t,dx)linearStatespace(A,t,dx,B,K),tspan,dx0);

% Inputs aren't saved, so recalculate here
dx = dx';
x = zeros(12,length(t));
u = zeros(4,length(t));
duty = zeros(4,length(t));
for i = 1:length(t)
    u(:,i) = -K*dx(:,i);
    x(:,i) = xe + dx(:,i);
    % Convert rotor speed to duty cycle 
    duty(:,i) = ((u(:,i)+p.rho*p.pi).^2 + 11.1*p.rho - (p.rho*p.pi)^2)./(222*p.rho);
    % This is a more convoluted way of writing PW = 800/v_max*(R*b/k_T*w^2
    % + k_e*w) + 1100 and then converting to % duty cycle
end

end