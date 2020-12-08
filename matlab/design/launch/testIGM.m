%Test IGM in this script
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
% vehicleparams.vehiclethrust_N = 15;
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

%vectors for plotting
theta = linspace(0,2*pi,100);
x_earth = R0*cos(theta);
y_earth = R0*sin(theta);
x_atmo = (R0+physparams.atmoheight_m)*cos(theta);
y_atmo = (R0+physparams.atmoheight_m)*sin(theta);
x_targ = RT*cos(theta);
y_targ = RT*sin(theta);

%time step
dt = vehicleparams.cycletime_sec;

%initial state
x0 = [0;
    physparams.earthradius_m + physparams.atmoheight_m;
    500;
    500;
    vehicleparams.vehiclemass_kg];

%initialize output
xhist = x0;
tvec = 0;


%loop managment
maxIdx = 10000;
idx = 1;
T2 = 3;

%loop
while(T2 > dt)
    
    %times
    t0 = tvec(idx);
    tf = t0 + dt;
    tvec = [tvec tf];
    
    %call IGM
    [chi_tilde, K1, PhiT, K2, T2] = IGM(xhist(1:4,idx), g0, R0, RT, VT, m12, Vex2, m2dot, xiT_dot, etaT_dot);
    
    if(abs(imag(T2)) > 0)
        disp("Imag time remaining!")
        break
    end
    
    %call segment propagator
    [~, xhistseg] = PropTrajSegmentIGM(xhist(:,idx),...
        t0, tf, physparams, vehicleparams,chi_tilde, K1, PhiT, K2, T2);
    
    %assign
    xhist = [xhist xhistseg(end,:)'];
    
    %update mass
    m12 = xhist(5,end);
    
    if(abs(imag(xhist(1,end))) > 0)
        disp("Imag position!")
        break
    end
    
    %break if necessary
    if(idx > maxIdx)
        disp("Max Idx")
        break;
    end
    
    if(xhist(5,end) < 0)
        disp("Out of Fuel")
        break;
    end
    
    if(norm(xhist(1:2,end)) < R0)
        disp("Crash!")
        break;
    end
    
    disp(T2)
    
    %increment
    idx = idx+1;
    
end

%propagate for awhile to see how our orbit looks


figure
plot(xhist(1,:),xhist(2,:))
hold on
plot(x_earth,y_earth,x_atmo,y_atmo,x_targ,y_targ)
legend('Trajectory','Earth','Atmosphere','Target Orbit')
% axis([-3*physparams.earthradius_m 3*physparams.earthradius_m -3*physparams.earthradius_m 3*physparams.earthradius_m])

figure
plot(tvec, xhist(5,:))

