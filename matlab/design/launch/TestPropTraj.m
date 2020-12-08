clear
close all
clc

%% Physics Parameters
physparams.earthradius_m = 6731000;
physparams.atmoheight_m = 100000; %karman line
physparams.earthrotation_rad_sec = 7.292*10^-5;
physparams.earthmass_kg = 5.972*10^24;
physparams.earthgrav = 3.98600441*10^14; %gravity parameter for earth

%% Vehicle Parameters
vehicleparams.vehiclemass_kg = 8000;
vehicleparams.coeffdrag = 0.75;
vehicleparams.effectivearea = 0.00001;
vehicleparams.Isp = 5000;
vehicleparams.m_dot = 10;
vehicleparams.cycletime_sec = .1;

%IGM Inputs
R0 = physparams.earthradius_m;
RT = R0 + 2.5*physparams.atmoheight_m;
g0 = -1*Grav(R0,physparams.earthgrav,1.0);
VT = sqrt(physparams.earthgrav/RT);
m12 = vehicleparams.vehiclemass_kg;
Vex2 = vehicleparams.Isp*g0;
m2dot = vehicleparams.m_dot;
xiT_dot = VT;
etaT_dot = 0;

%time step
dt = vehicleparams.cycletime_sec;
t = 0:dt:300;
L = length(t);

%generate time tilt polynomial
t_exo = 70;
timetiltpoly = GenInitPoly(t_exo);

%initial state
x0 = [0;
    physparams.earthradius_m;
    0;
    0;
    vehicleparams.vehiclemass_kg];

%propagate trajectory
xhist = PropTraj(t, timetiltpoly, x0, physparams, vehicleparams);

%calc alt
alt = zeros(L,1);
for ii = 1:L
    alt(ii) = norm(xhist(1:2,ii)) - physparams.earthradius_m;
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

figure
plot(t, alt)