function [t,x,u,pulsewidth] = nlSim(x0,tspan,K,ue,xe,p)

% Simulate nonlinear system
[t,x] = ode45(@(t, x) NonlinearStatespace(t,x,K,ue,xe,p), tspan, x0);

% Inputs aren't saved in simulation, so recalculate them here
u    = zeros(4,length(t));
pulsewidth = zeros(4,length(t));
for i = 1:length(t)
    u(:,i)    = ue'-K*(x(i,:)'-xe');
    u(u>p.max_omega) = p.max_omega;
    u(u<p.min_omega) = p.min_omega;
    % Convert rotor speed to pulsewidth
    pulsewidth(:,i) = 800/(p.rho*p.vmax)*((u(:,i)+p.rho*p.pi).^2 - (p.rho*p.pi)^2) + 1100;
    % This is a more convoluted way of writing PW = 800/v_max*(R*b/k_T*w^2 + k_e*w) + 1100
end

end