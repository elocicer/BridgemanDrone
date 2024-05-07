% PlotDroneData
close all
clear
clc
savetime = clock;
Data = load('data.csv');
fig_filename = 'ExperimentPlots/HandTest1/';
fig_filetime = ['__',num2str(savetime(1)),'_',num2str(savetime(2)),'_',num2str(savetime(3)),'_',num2str(savetime(4)),'_',num2str(savetime(5))];

% Separate data
N      = size(Data,1);
t      = Data(:,1);
x      = Data(:,2);
y      = Data(:,3);
z      = Data(:,4);
roll   = Data(:,5);
pitch  = Data(:,6);
yaw    = Data(:,7);
dx     = Data(:,8);
dy     = Data(:,9);
dz     = Data(:,10);
droll  = Data(:,11);
dpitch = Data(:,12);
dyaw   = Data(:,13);
u1     = Data(:,14);
u2     = Data(:,15);
u3     = Data(:,16);
u4     = Data(:,17);
deltax      = Data(:,18);
deltay      = Data(:,19);
deltaz      = Data(:,20);
deltaroll   = Data(:,21);
deltapitch  = Data(:,22);
deltayaw    = Data(:,23);
deltadx     = Data(:,24);
deltady     = Data(:,25);
deltadz     = Data(:,26);
deltadroll  = Data(:,27);
deltadpitch = Data(:,28);
deltadyaw   = Data(:,29);
yaw_looper  = Data(:,30);
rawyaw      = Data(:,31);

% Calculate average bandwidth
Hz = ((t(end)-t(1))/N)^-1

% Calculat Latency at each step
dt = zeros(N-1,1);
for index = 2:N
    dt(index) = t(index) - t(index-1);
end
fig_latency = figure;
histogram(dt)

% Idealtime
t_ideal = linspace(t(1),t(end),N);

% Convert to relative time
t_rel = t-t(1);
%{
% Calculate low-pass filter for x
T_pos_filter  = .1;
K_pos_filter  = 1;
xfilterstate  = zeros(N,1);
yfilterstate  = zeros(N,1);
zfilterstate  = zeros(N,1);
filtered_x    = zeros(N-1,1);
filtered_y    = zeros(N-1,1);
filtered_z    = zeros(N-1,1);
dxfilterstate = zeros(N,1);
dyfilterstate = zeros(N,1);
dzfilterstate = zeros(N,1);
T_vel_filter  = .25;
K_vel_filter  = 1;
filtered_dx   = zeros(N-1,1);
filtered_dy   = zeros(N-1,1);
filtered_dz   = zeros(N-1,1);
for index = 1:N-1
    xfilterstate(index+1)  = (1-dt(index)/T_pos_filter)*xfilterstate(index) + K_pos_filter*dt(index)/T_pos_filter*x(index);
    yfilterstate(index+1)  = (1-dt(index)/T_pos_filter)*yfilterstate(index) + K_pos_filter*dt(index)/T_pos_filter*y(index);
    zfilterstate(index+1)  = (1-dt(index)/T_pos_filter)*zfilterstate(index) + K_pos_filter*dt(index)/T_pos_filter*z(index);
    filtered_x(index)      = xfilterstate(index);
    filtered_y(index)      = yfilterstate(index);
    filtered_z(index)      = zfilterstate(index);
    dxfilterstate(index+1) = (1-dt(index)/T_vel_filter)*dxfilterstate(index) + K_vel_filter*dt(index)/T_vel_filter*dx(index);
    dyfilterstate(index+1) = (1-dt(index)/T_vel_filter)*dyfilterstate(index) + K_vel_filter*dt(index)/T_vel_filter*dy(index);
    dzfilterstate(index+1) = (1-dt(index)/T_vel_filter)*dzfilterstate(index) + K_vel_filter*dt(index)/T_vel_filter*dz(index);
    filtered_dx(index)     = dxfilterstate(index);
    filtered_dy(index)     = dyfilterstate(index);
    filtered_dz(index)     = dzfilterstate(index);
end

figure
hold on
plot(t_rel,x)
plot(t_rel(1:end-1),filtered_x)
hold off

figure
hold on
plot(t_rel,y)
plot(t_rel(1:end-1),filtered_y)
hold off

figure
hold on
plot(t_rel,z)
plot(t_rel(1:end-1),filtered_z)
hold off

figure
hold on
plot(t_rel,dx)
plot(t_rel(1:end-1),filtered_dx)
hold off

figure
hold on
plot(t_rel,dy)
plot(t_rel(1:end-1),filtered_dy)
hold off

figure
hold on
plot(t_rel,dz)
plot(t_rel(1:end-1),filtered_dz)
hold off

dx_conditioned = zeros(N-2,1);
dy_conditioned = zeros(N-2,1);
dz_conditioned = zeros(N-2,1);
for index = 2:N-1
    dx_conditioned(index) = (filtered_x(index) - filtered_x(index-1))/dt(index);
    dy_conditioned(index) = (filtered_y(index) - filtered_y(index-1))/dt(index);
    dz_conditioned(index) = (filtered_z(index) - filtered_z(index-1))/dt(index);
end

figure
hold on
plot(t_rel,dx)
plot(t_rel(2:end),dx_conditioned)
hold off

figure
hold on
plot(t_rel,dy)
plot(t_rel(2:end),dy_conditioned)
hold off

figure
hold on
plot(t_rel,dz)
plot(t_rel(2:end),dz_conditioned)
hold off
%}
%% Plot
fig_position = figure;
title('Position')
subplot(2,1,1)
hold on
plot(t_rel,x)
plot(t_rel,y)
plot(t_rel,z)
legend('x','y','z')
xlabel('time (s)')
ylabel('position (m)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,dx)
plot(t_rel,dy)
plot(t_rel,dz)
legend('dx','dy','dz')
xlabel('time (s)')
ylabel('velocity (m/s)')
set(gca,'FontSize',18)
hold off

fig_attitude = figure;
title('Attitude')
subplot(2,1,1)
hold on
plot(t_rel,roll)
plot(t_rel,pitch)
plot(t_rel,yaw)
plot(t_rel,rawyaw)
plot(t_rel,yaw_looper)
legend('roll','pitch','yaw','rawyaw','yaw_{looper}')
xlabel('time (s)')
ylabel('attitude (rad)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,droll)
plot(t_rel,dpitch)
plot(t_rel,dyaw)
legend('droll','dpitch','dyaw')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
set(gca,'FontSize',18)
hold off

fig_controls = figure;
title('Controls')
hold on
plot(t_rel,u1)
plot(t_rel,u2)
plot(t_rel,u3)
plot(t_rel,u4)
legend('u1','u2','u3','u4')
xlabel('time (s)')
ylabel('pulsewidth (micro s)')
set(gca,'FontSize',18)
hold off

%% Plot Deltas
fig_relposition = figure;
title('Position From Reference')
subplot(2,1,1)
hold on
plot(t_rel,deltax)
plot(t_rel,deltay)
plot(t_rel,deltaz)
legend('\Delta x','\Delta y','\Delta z')
xlabel('time (s)')
ylabel('position (m)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,deltadx)
plot(t_rel,deltady)
plot(t_rel,deltadz)
legend('\Delta dx','\Delta dy','\Delta dz')
xlabel('time (s)')
ylabel('velocity (m/s)')
set(gca,'FontSize',18)
hold off

fig_relattitude = figure;
title('Attitude From Reference')
subplot(2,1,1)
hold on
plot(t_rel,deltaroll)
plot(t_rel,deltapitch)
plot(t_rel,deltayaw)
legend('\Delta roll','\Delta pitch','\Delta yaw')
xlabel('time (s)')
ylabel('attitude (rad)')
set(gca,'FontSize',18)
hold off
subplot(2,1,2)
hold on
plot(t_rel,deltadroll)
plot(t_rel,deltadpitch)
plot(t_rel,deltadyaw)
legend('\Delta droll','\Delta dpitch','\Delta dyaw')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
set(gca,'FontSize',18)
hold off

saveas(fig_attitude,[fig_filename,'position',fig_filetime])
saveas(fig_position,[fig_filename,'attitude',fig_filetime])
saveas(fig_relattitude,[fig_filename,'relposition',fig_filetime])
saveas(fig_relposition,[fig_filename,'relattitude',fig_filetime])
saveas(fig_controls,[fig_filename,'controls',fig_filetime])
saveas(fig_latency, [fig_filename,'latency' ,fig_filetime])




