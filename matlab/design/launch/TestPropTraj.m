clear
close all
% clc

%% Physics Parameters
physparams.earthradius_m = 6731000;
physparams.atmoheight_m = 100000; %karman line
physparams.earthrotation_rad_sec = 7.292*10^-5;
physparams.earthmass_kg = 5.972*10^24;
physparams.earthgrav = 3.98600441*10^14; %gravity parameter for earth

%% Vehicle Parameters
vehicleparams.vehiclemass_kg = 8000;
vehicleparams.coeffdrag = 0.75;
vehicleparams.effectivearea = pi*2^2;
vehicleparams.Isp = 5000;
vehicleparams.g0 = 9.81;
vehicleparams.m_dot_max = 10;
vehicleparams.cycletime_sec = 1;
vehicleparams.Kp = 1000; %atmo PID speed controller params
vehicleparams.Ki = 500; %atmo PID speed controller params
vehicleparams.Tmax = vehicleparams.Isp*vehicleparams.g0*vehicleparams.m_dot_max;
vehicleparams.Tmin = 0.25*vehicleparams.Tmax;
vehicleparams.useheightpoly = true;

%IGM Inputs
R0 = physparams.earthradius_m;
RT = R0 + 2.5*physparams.atmoheight_m;
g0 = -1*Grav(R0,physparams.earthgrav,1.0);
VT = sqrt(physparams.earthgrav/RT);
m12 = vehicleparams.vehiclemass_kg;
Vex2 = vehicleparams.Isp*g0;
m2dot = vehicleparams.m_dot_max;
xiT_dot = VT;
etaT_dot = 0;

%time step
dt = vehicleparams.cycletime_sec;
t = [0:dt:1500, 1500+dt:100:8000];
L = length(t);

%generate time tilt polynomial
t_exo = 310;
timetiltpoly = GenInitPoly(t_exo);
Nsegment = 1; %number of segments for height tilt poly
Ndegree = 2; %number of terms in height tilt poly. Const plus lin is 2 terms
heighttiltpoly = GenInitHeightTiltPoly(physparams.atmoheight_m,Nsegment,Ndegree); 

%initial state
x0 = [0;
    physparams.earthradius_m;
    0;
    0;
    vehicleparams.vehiclemass_kg];

%propagate trajectory
xhist = PropTraj(t, heighttiltpoly, x0, physparams, vehicleparams);

%calc alt, speed, Vt
alt = zeros(L,1);
speed = zeros(L,1);
Vt = zeros(L,1);
for ii = 1:L
    alt(ii) = norm(xhist(1:2,ii)) - physparams.earthradius_m;
    speed(ii) = norm(xhist(3:4,ii));
    if(alt(ii) < physparams.atmoheight_m)
        m = xhist(5,ii);
        g = norm(Grav(xhist(1:2,ii),physparams.earthgrav,1));
        rho = AirDensity(alt(ii),physparams.atmoheight_m);
        Vt(ii) = sqrt((2*m*g)/(rho*vehicleparams.effectivearea*vehicleparams.coeffdrag));
    end
end

%% Plotting

%vectors for plotting
theta = linspace(0,2*pi,100);
x_earth = R0*cos(theta);
y_earth = R0*sin(theta);
x_atmo = (R0+physparams.atmoheight_m)*cos(theta);
y_atmo = (R0+physparams.atmoheight_m)*sin(theta);
x_targ = RT*cos(theta);
y_targ = RT*sin(theta);

figure
plot(xhist(1,:),xhist(2,:),'LineWidth',2)
hold on
plot(x_earth,y_earth,x_atmo,y_atmo,x_targ,y_targ,'x')
legend('Trajectory','Earth','Atmosphere','Target Orbit')

figure
plot(t, xhist(5,:))
xlabel('time')
ylabel('vehicle mass')

figure
plot(t, alt)
xlabel('Time')
ylabel('vehicle altitude')

figure
plot(t,speed,t,Vt)
xlabel('time')
ylabel('speed (m/s)')
legend('Vehicle Speed','Terminal Velocity','location','best')

disp('Final Mass:')
disp(xhist(5,end))