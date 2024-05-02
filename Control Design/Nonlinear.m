function [x_sim, t_sim, e_sim] = Nonlinear(xo, uo, t_range)

% xo: intital state
% uo: initial motor speeds
% t_range: time length to simulate

m =1; % mass of drone
k = 1; % lift constant
b = 1; % drag constant
g = 9.8; 
l = 1; % distance between rotor and center of mass of quadcopter
Ixx = 1;
Iyy = 1; % equal to Ixx (probably)
Izz = 1;
Im = 1; % moment of inertia of motor

p.m = m;
p.Ixx = Ixx;
p.Iyy = Iyy;
p.Izz = Izz;
p.g = g;
p.Im = Im;

[t_sim, x_sim] = ode45(@(t_sim, x) statespace(t_sim, x, uo), [0 t_range], xo);

e_sim = getenergy(x_sim, p, uo);

end
%% Energy Fxn 
function E = energy(x, p, u) % input as column vector

transdot = x(7:9);

etrans = p.m/2*(transdot')*transdot;
% etrans is correct

W = [1 0 -sin(x(5));
    0 cos(x(4)) cos(x(5))*sin(x(4));
    0 -sin(x(4)) cos(x(5))*cos(x(4))];
nu = W * x(10:12);
I = [p.Ixx 0 0; 0 p.Iyy 0; 0 0 p.Izz];
erot = 1/2*(nu')*I*nu;

epot = p.m*p.g*x(3);

emotors = 1/2*p.Im*(u(1)^2+u(2)^2+u(3)^2+u(4)^2);

E = epot+etrans+emotors+erot;
end
%% GetE Fxn
function energies = getenergy(x,p,u)

x=x';
num_times = width(x);

energies = zeros(num_times,1);
for i= 1:num_times
    energies(i) = energy(x(:,i),p,u);
end
end
%% Plant Fxn
function xdot = statespace(t,x,u)
% A = [0 1; -2 -1];
% dydt = A*(x);

m =1; % mass of drone
k = 1; % lift constant
b = 1; % drag constant
g = 9.8; 
l = 1; % distance between rotor and center of mass of quadcopter
Ixx = 1;
Iyy = 1; % equal to Ixx (probably)
Izz = 1;

T = k * (u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2);

xdot(1) = x(7);
xdot(2) = x(8);
xdot(3) = x(9);
xdot(4) = x(10);
xdot(5) = x(11);
xdot(6) = x(12);

xdot(7) = T / m * (cos(x(6)) * sin(x(5)) * cos(x(4)) + sin(x(6)) * sin(x(4)));
xdot(8) = T / m * (sin(x(6)) * sin(x(5)) * cos(x(4)) + cos(x(6)) * sin(x(4)));
xdot(9) = -g + T / m * (cos(x(5)) * cos(x(4)));


C11 = 0;
C12 = (Iyy - Izz)*(x(11)*cos(x(4))*sin(x(4)) + x(12)*(sin(x(4))^2)*cos(x(5))) + (Izz - Iyy)*(x(12)*cos(x(4))^2*cos(x(5))) - Ixx*x(12)*cos(x(5));
C13 = (Izz - Iyy)*x(12)*cos(x(4))*sin(x(4))*(cos(x(5))^2);
C21 = (Izz - Iyy)*(x(11)*cos(x(4))*sin(x(4)) + x(12)*sin(x(4))*cos(x(5))) + (Iyy - Izz)*(x(12)*cos(x(4))^2*cos(x(5))) + Ixx*x(12)*cos(x(5));
C22 = (Izz - Iyy)*x(10)*cos(x(4))*sin(x(4));
C23 = -Ixx*x(12)*sin(x(5))*cos(x(5)) + Iyy*x(12)*sin(x(4))^2*sin(x(5))*cos(x(5)) + Izz*x(12)*cos(x(4))^2*sin(x(5))*cos(x(5));
C31 = (Iyy - Izz)*x(12)*cos(x(5))^2*sin(x(4))*cos(x(4)) - Ixx*x(11)*cos(x(5));
C32 = (Izz - Iyy)*(x(11)*cos(x(4))*sin(x(4))*sin(x(5)) + x(10)*sin(x(4))^2*cos(x(5))) + (Iyy - Izz)*(x(10)*cos(x(4))^2*cos(x(5))) + Ixx*x(12)*cos(x(5))*sin(x(5)) - Iyy*x(12)*sin(x(4))^2*sin(x(5))*cos(x(5)) - Izz*x(12)*cos(x(4))^2*sin(x(5))*cos(x(5));
C33 = (Iyy - Izz)*x(10)*cos(x(4))*sin(x(4))*cos(x(5))^2 - Iyy*x(11)*sin(x(4))^2*cos(x(5))*sin(x(5)) - Izz*x(11)*cos(x(4))^2*cos(x(5))*sin(x(5)) + Ixx*x(11)*cos(x(5))*sin(x(5));

J11 = Ixx;
J12 = 0;
J13 = -Ixx*sin(x(5));
J21 = 0; 
J22 = Iyy*cos(x(4))^2 + Izz*sin(x(4))^2;
J23 = (Iyy - Izz)*cos(x(4))*sin(x(4))*cos(x(5));
J31 = J13;
J32 = J23;
J33 = Ixx*sin(x(5))^2 + Iyy*(sin(x(4))^2)*cos(x(5))^2 + Izz*(cos(x(4))^2)*cos(x(5))^2;

J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];


C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
ndot = [x(10) x(11) x(12)]';
tb1 = l*k*(-u(2)^2 + u(4)^2);
tb2 = l*k*(-u(1)^2 + u(3)^2);
tb3 = b*(-u(1)^2 + u(2)^2 - u(3)^2 + u(4)^2);
tb = [tb1 tb2 tb3]';

ndotdot = inv(J) * (tb - (C*ndot)); % / does inverse faster 
xdot(10) = ndotdot(1);
xdot(11) = ndotdot(2);
xdot(12) = ndotdot(3);
xdot=xdot';
end