% This matlab script serves as a prototyping ground for developing my
% launch ascent guidance system, it will be poorly documented, better
% documentation for the final system will be found in the repository's
% documentation

%The general plan is to find a minimum fuel ascent trajactory which targets
%some specified circular orbit. This will all be done in two parts,
%following the example of the Saturn V guidance system. The atmospheric
%portion of flight is controlled by a time tilt program where we ascend
%maintaining terminal velocity. (I will show this is roughly optimal in the
%documentation). The exo-atmospheric portion is controlled via Saturn's IGM
%algorithm.

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
vehicleparams.effectivearea = pi*2^2;
vehicleparams.Isp = 5000;
vehicleparams.g0 = 9.81;
vehicleparams.m_dot_max = 10;
vehicleparams.cycletime_sec = 1;
vehicleparams.Kp = 1000; %atmo PID speed controller params
vehicleparams.Ki = 500; %atmo PID speed controller params
vehicleparams.Tmax = vehicleparams.Isp*vehicleparams.g0*vehicleparams.m_dot_max;
vehicleparams.Tmin = 0.25*vehicleparams.Tmax;
vehicleparams.PolySeg = 1;
vehicleparams.PolyDeg = 2;
vehicleparams.useheightpoly = true;

%% IGM Inputs
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
t = 0:dt:750;
L = length(t);

%% Optimization Parameters

matoptions = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'Display','iter-detailed','FiniteDifferenceStepSize',1E-3,...
    'MaxIterations',100,'FiniteDifferenceType','central',...
    'FunctionTolerance',1E-8);


%% Main

%initial state
x0 = [0;
    physparams.earthradius_m;
    0;
    0;
    vehicleparams.vehiclemass_kg];

%initial estimate for polynomial
if(vehicleparams.useheightpoly)
    poly = GenInitHeightTiltPoly(physparams.atmoheight_m, ...
        vehicleparams.PolySeg, vehicleparams.PolyDeg);
    %poly.coeffs(2,1) = -0.0808;
    dvar_init = HeightPolyToVect(poly);
else
    t_tilt_end = 310; %seconds
    poly = GenInitPoly(t_tilt_end);
    dvar_init = PolyToVect1stDeg(poly);
end

%form function
fun = @(u) TrajCost(u, t, x0, physparams, vehicleparams);
% funJac = @(u) FD(u,fun,1E-3);
% jac = funJac(dvar_init);

Nsweep = 25;
dvar_sweep = linspace(1.01*dvar_init,1.02*dvar_init,Nsweep);
cost_sweep = zeros(Nsweep,1);
for ii = 1:Nsweep
    cost_sweep(ii) = fun(dvar_sweep(ii));
    disp(ii)
end

figure
plot(dvar_sweep,cost_sweep)

%call matlab solver
dvar_f = lsqnonlin(fun, dvar_init, [], [], matoptions);

