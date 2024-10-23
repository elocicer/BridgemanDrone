% LiftData
close all

%% Info
% Lift force and rotor speed were measured in response to a series of PWM
% signals, one motor at a time. The lift force was measured by securing the
% motor to a scale and recording the weight difference. Rotor speed was
% measured using a laser tachometer. At each data point, the battery 
% voltage was also recorded. Each motor was tested once around 12.2 and 
% again around 11.2 volts. Motor and rotor parameters are estimated by
% fitting theoretical formulae to the data.

%% Data
% Notation: 
% w       -> rotor speed
% F       -> lift force
% Vmax    -> battery voltage
% V       -> voltage delivered to motor
% 1,2,3,4 -> motor number
% h,l     -> high volt experiment or low volt experiment

% PWM commands, microseconds
PW = [1100 1150 1250 1350 1450  1550 1650];

% Measured rotor speeds, rpm
w1h = [0    1270 2360 3286 4253  5090 5902]; 
w1l = [0    1195 2217 3073 3985  4769 5580];
w2h = [0    1446 2560 3409 4370  5228 6055];
w2l = [0    1444 2408 3171 4067  4859 5688];
w3h = [0    1479 2478 3372 4310  5160 5996];
w3l = [0    1403 2359 3150 4050  4821 5624];
w4h = [0    1466 2544 3421 4368  5200 6000];
w4l = [0    1170 2174 3078 3970  4790 5653];

% Lift force, converted from g to N
F1h = [0    20   68   135  233   345  463 ]/1000*9.81;
F1l = [0    12   56.5 116  204   295  420 ]/1000*9.81;
F2h = [0    22   76   143  242   357  494 ]/1000*9.81;
F2l = [0    21   66   123  212   313  440 ]/1000*9.81;
F3h = [0    25.5 73   141  242   344  480 ]/1000*9.81;
F3l = [0    19.4 63   120  206.5 304  420 ]/1000*9.81;
F4h = [0    19   70   139  240   353  489 ]/1000*9.81;
F4l = [0    14   53   112  196   298  420 ]/1000*9.81;

% battery voltage, Volts
Vmax1h = [12.3 12.3 12.2 12.2 12.2 12.1 12.1];
Vmax1l = [11.3 11.3 11.3 11.3 11.3 11.2 11.2];
Vmax2h = [12.3 12.3 12.3 12.3 12.2 12.2 12.2];
Vmax2l = [11.3 11.3 11.3 11.3 11.2 11.2 11.2];
Vmax3h = [12.2 12.2 12.2 12.2 12.1 12.1 12.1];
Vmax3l = [11.3 11.3 11.3 11.3 11.2 11.2 11.1];
Vmax4h = [12.5 12.5 12.4 12.4 12.4 12.3 12.2];
Vmax4l = [11.3 11.3 11.3 11.2 11.2 11.2 11.1];

% Voltage delivered to motors
% V = Vmax/800*(PW-1100)
% 1100 is the minimum pulsewidth, and 800 is the range of pulsewidths
V1h = Vmax1h.*(PW-1100)./800;
V1l = Vmax1l.*(PW-1100)./800;
V2h = Vmax2h.*(PW-1100)./800;
V2l = Vmax2l.*(PW-1100)./800;
V3h = Vmax3h.*(PW-1100)./800;
V3l = Vmax3l.*(PW-1100)./800;
V4h = Vmax4h.*(PW-1100)./800;
V4l = Vmax4l.*(PW-1100)./800;

%% Fit lift coefficient
% F = kw^2
% Find linear regression on F vs w^2, constrained with (0,0)
figure
hold on
plot(w1h.^2,F1h,'kx')
plot(w2h.^2,F2h,'rx')
plot(w3h.^2,F3h,'bx')
plot(w4h.^2,F4h,'gx')
plot(w1l.^2,F1l,'ko')
plot(w2l.^2,F2l,'ro')
plot(w3l.^2,F3l,'bo')
plot(w4l.^2,F4l,'go')
wSquared = [w1h, w2h, w3h, w4h, w1l, w2l, w3l, w4l].^2;
F        = [F1h, F2h, F3h, F4h, F1l, F2l, F3l, F4l];
fitfun1  = fittype(@(k,x) k*x);
k0       = 1e-7;
fit1     = fit(wSquared',F',fitfun1,'StartPoint',k0);
t1 = linspace(min(wSquared),max(wSquared),100);
y2 = fit1.k*t1;
plot(t1,y2,'r-')
hold off
k = fit1.k;

%% Fit drag coefficient and motor parameters
% v = Rb/kT*w^2 + ke*w
% Fit the above function, with parameters RbkT = Rb/kT and ke.
% The above function comes from the motor dynamics
% v      = R*i + L*i_dot + ke*w
% Jw_dot = -kT*i - bw^2
% evaluated at equilibrium.

figure
hold on
plot(w1h,V1h,'kx')
plot(w2h,V2h,'rx')
plot(w3h,V3h,'bx')
plot(w4h,V4h,'gx')
plot(w1l,V1l,'ko')
plot(w2l,V2l,'ro')
plot(w3l,V3l,'bo')
plot(w4l,V4l,'go')
w = [w1h, w1l, w2h, w2l, w3h, w3l, w4h, w4l];
V = [V1h, V1l, V2h, V2l, V3h, V3l, V4h, V4l];
fitfun2 = fittype(@(RbkT, ke, x) RbkT*x.^2 + ke*x);
RbkT0   = 1/(8e6);
ke0     = 0.0005;
fit2    = fit(w',V',fitfun2,'StartPoint',[RbkT0; ke0]);
t2 = linspace(min(w),max(w),100);
y2 = fit2.RbkT*t2.^2 + fit2.ke*t2;
plot(t2,y2,'r-')
hold off
ke = fit2.ke;

%% Measure motor resistance experimentally
% Method: https://www.youtube.com/watch?v=4GgE01acZ6E\&ab\_channel=RCexplained
R = .17;

%% Solve for remaining terms
% ke and kT are directly related. ke = V/w, and kT = torque/i. Electrical
% and mechanical power balance gives 
% sqrt(3)*v*i = 2*pi/60*w*torque
% where sqrt(3)*v*i is rms electrical power, and 2*pi/60 converts
% mechanical power, w*torque, to Nm/s. Rearranging yields the result.
kT = 60*sqrt(3)/(2*pi)*ke;
b  = fit2.RbkT/R*kT;

%% Print results
disp(['R: ',num2str(R)])
disp(['k: ',num2str(k)])
disp(['ke: ',num2str(ke)])
disp(['kT: ',num2str(kT)])
disp(['b: ',num2str(b)])

%% Conversion from rotor speed to PW
% PW = 800/Vmax*(R*b/kT*w^2 + ke*w) + 1100

