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
earthradius_m = 6731000;
atmosphereheight_m = 100000; %karman line
earthrotation_rad_sec = 7.292*10^-5;
earthmass_kg = 5.972*10^24;
earthgrav = 3.98600441*10^14; %gravity parameter for earth

%% Vehicle Parameters
vehiclemass_kg = 1;
vehiclethrust_N = 50000;
coeffdrag = 0.75;

%% Optimization Parameters
numnodes = 1000; %number of discrete nodes for optimization

%% Main

% disp("Something seems wrong with my gravity???")
% grav([earthradius_m,0]',earthgrav,vehiclemass_kg)

test = zeros(1000,1);
alt = linspace(1,atmosphereheight_m,1000);
for ii = 1:1000
    test(ii) = AirDensity(alt(ii));
end

figure
plot(alt,test)




