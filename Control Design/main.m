close all
clear

%% Parameters
p.g   = 9.81;       % gravitational constant, m/s^2
p.m   = 0.9445;     % mass of drone, kg
p.l   = .23;        % distance between rotor and center of mass of quadcopter
p.Ixx = 0.013022;   % moment of inertia around x axis, kg*m^2
p.Iyy = 0.012568;   % moment of inertia around y axis, kg*m^2
p.Izz = 0.021489;   % moment of inertia around z axis, kg*m^2
p.k   = 1.29e-7;    % lift constant, N/rpm
p.b   = 8.21e-9;        % drag constant, N/rpm
p.kT  = 0.0108;     % torque coefficient of motor
p.b   = 8.21e-9;    % drag coefficient of rotor
p.R   = 0.17;       % motor resistance
p.ke  = 0.000656;   % back EMF constant    
p.vmax = 11.5;      % battery voltage (bit of a misnomer; it's the current battery voltage, not the max battery voltage)
p.min_PW  = 1100;   % max pulsewidth signal
p.max_PW  = 1900;   % min pulsewidth signal
p.rho       = p.kT/(p.b*p.R);
p.pi        = p.ke/2; 
p.max_omega = sqrt(p.rho*p.vmax/800*(p.max_PW-1100) + (p.rho*p.pi)^2) - p.rho*p.pi; % max rotor speed
p.min_omega = 0; % min rotor speed

%% LQR Control Design
xe = [0 0 .5 0 0 0 0 0 0 0 0 0];         % State equilibrium
u_num = sqrt(p.m*p.g/p.k)/4;            % Nominal force to offset gravity
ue = 2*[u_num u_num u_num u_num];       % Control equilibrium
[A,B] = linearize(xe, ue, p);           % Generate Linearized System Model
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);    % State Weight
R = 0.000001.*eye(4);                   % Control Weight
K = lqr(A,B,Q,R);                       % Feedback Matrix
save('Controllers/LQRcontroller','K','p','ue')

%% Simulations
tspan = [0 5];                                            % time range
x0    = xe + [0 0 .1 .1 .1 .1 0 0 0 0 0 0];                % initial conditions
[t_lin, x_lin, u_lin, duty_lin] = LinearSim(tspan, xe', x0', A,B,K,p); % linear simulation
[t_nl, x_nl, u_nl, duty_nl]     = nlSim(x0',tspan,K,ue,xe,p);        % nonlinear simulation

%% Plots
figure(1)
plot(t_lin,x_lin(1:6,:))
title('Linear Simulation at Equilibrium')
legend('x','y','z','phi','theta','gamma')

figure(2)
subplot(3,1,1)
plot(t_nl,x_nl(:,1:6))
title('Nonlinear Simulation at Equilibrium')
legend('x','y','z','phi','theta','gamma')
subplot(3,1,2)
plot(t_nl,u_nl)
legend('\omega_1','\omega_2','\omega_3','\omega_4')
ylim([p.min_omega p.max_omega])
subplot(3,1,3)
plot(t_nl,duty_nl)
legend('PW_1','PW_2','PW_3','PW_4')
ylim([p.min_PW p.max_PW])

