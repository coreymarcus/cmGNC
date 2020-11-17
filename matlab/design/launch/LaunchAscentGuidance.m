% This matlab script serves as a prototyping ground for developing my
% launch ascent guidance system, it will be poorly documented, better
% documentation for the final system will be found in the repositories
% documentation

%The general plan is to find a minimum fuel ascent trajactory which targets
%some specified circular orbit. This will all be done through direct
%optimization methods.

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
vehicleparams.vehiclemass_kg = 1;
vehicleparams.vehiclethrust_N = 50000;
vehicleparams.coeffdrag = 0.75;
vehicleparams.effectivearea = 0;

%% Optimization Parameters
numnodes = 1000; %number of discrete nodes for optimization

%% Main

% Test for gravity
% disp("Something seems wrong with my gravity???")
% grav([earthradius_m,0]',earthgrav,vehiclemass_kg)

% test for air pressure
% test = zeros(1000,1);
% alt = linspace(1,atmosphereheight_m,1000);
% for ii = 1:1000
%     test(ii) = AirDensity(alt(ii));
% end
% 
% figure
% plot(alt,test)

% test a trajectory propagator
x0 = zeros(5,1);
x0(2) = physparams.earthradius_m;
x0(3) = 0;
x0(5) = 1;
t = 0:.1:15;
N = length(t);
u = [linspace(1,50,N);
    linspace(1,50,N)];
xhist = PropTraj(t, u, x0, physparams, vehicleparams);

figure
plot(xhist(1,:), xhist(2,:))
axis equal
grid on




