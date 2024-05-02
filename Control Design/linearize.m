function [A,B] = linearize(xe, ue, p)
%% Constants 
m =p.m;
k = p.k; 
b = p.b; 
g = p.g; 
l = p.l;
Ixx = p.Ixx;
Iyy = p.Iyy; 
Izz = p.Izz;

%% Symbolic Nonlinear Dynamics
x = sym('x', [1 12]);
u = sym('u',[1 4]);
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
C12 = (Iyy - Izz)*(x(11)*cos(x(4))*sin(x(4)) + x(12)*sin(x(4))^2*cos(x(5))) + (Izz - Iyy)*(x(12)*cos(x(4))^2*cos(x(5))) - Ixx*x(12)*cos(x(5));
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
J33 = Ixx*sin(x(5))^2 + Iyy*sin(x(4))^2*cos(x(5))^2 + Izz*cos(x(4))^2*cos(x(5))^2;

J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];


C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];
ndot = [x(10) x(11) x(12)]';
tb1 = l*k*(-u(2)^2 + u(4)^2);
tb2 = l*k*(-u(1)^2 + u(3)^2);
tb3 = b*(-u(1)^2 + u(2)^2 - u(3)^2 + u(4)^2);
tb = [tb1 tb2 tb3]';

ndotdot = inv(J) * (tb - C*ndot); % / does inverse faster 
xdot(10:12) = ndotdot;

%% Symbolically Differentiate
% Finding A matrix
A = sym('A',[12 12]); 

for r = 1:12
    for c = 1:12
            A(r,c) = diff(xdot(r),x(c),1);
    end
end

% Finding B matrix
B = sym('B', [12 4]);

for r = 1:12
    for c = 1:4
            B(r,c) = diff(xdot(r),u(c),1);
    end
end
% Linearize about eq point analytically  
A = subs(A, x, xe);
B = subs(B, x, xe);
A = subs(A, u, ue);
B = subs(B, u, ue);

A = double(A);
B = double(B);

end
