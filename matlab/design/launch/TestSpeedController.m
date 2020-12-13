%Tests the simulink speed controller
clear
close all
clc

%define constants
atmo_height = 100000;
Cd = 0.75;
A = pi*2^2;
m0 = 8000;
g0 = 9.81;
Isp = 50000;
SpeedSet = 10;
tf = 1000; %simulation time
Tmax = 5*g0*m0;
Tmin = 0.5*g0*m0;
mu_earth = 3.98600441*10^14;
radius_earth = 6731000;

%tune filter
Kp = 1000;
Ki = 500;
Kd = 0;

%run sim
sim SpeedController


%plot
figure
subplot(2,1,1)
plot(t_out, v_out, t_out, vterm_out)
legend('Velocity','Terminal Velocity')
xlabel('t (sec)')
ylabel('vel (m/s)')
subplot(2,1,2)
plot(t_out,T_out)
ylabel('Thrust (N)')
xlabel('t (sec)')

figure
plot(t_out, a_out/g0 + 1)
xlabel('time (sec)')
ylabel('Accel (g)')

% ylabel('accel (g)')
% xlabel('t (sec)')
% 
% figure
% plot(t_out, h_out)
% ylabel('height (m)')