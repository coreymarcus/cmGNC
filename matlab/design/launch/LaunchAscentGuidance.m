% This matlab script serves as a prototyping ground for developing my
% launch ascent guidance system, it will be poorly documented, better
% documentation for the final system will be found in the repository's
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
vehicleparams.effectivearea = 0.00001;

%% Optimization Parameters
numnodes = 1000; %number of discrete nodes for optimization

%% Other Setup
addpath('C:\Users\corey\Documents\GitHub\matlabScripts')
addpath('C:\Users\corey\Documents\GitHub\OST\Project')

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
% x0 = zeros(5,1);
% x0(2) = physparams.earthradius_m;
% x0(3) = 0;
% x0(5) = 1;
% t = 0:.1:15;
% N = length(t);
% u = [linspace(1,50,N);
%     linspace(1,50,N)];
% xhist = PropTraj(t, u, x0, physparams, vehicleparams);
% 
% figure
% plot(xhist(1,:), xhist(2,:))
% axis equal
% grid on

% test initial trajectory generation
N = 500;
rtarg = physparams.earthradius_m + 2.5*physparams.atmoheight_m;
x0 = [0 physparams.earthradius_m]';
traj = GenerateInitialTrajectory(N, rtarg, x0, physparams, vehicleparams);

%initial state
initstate = [x0; 0; 0; vehicleparams.vehiclemass_kg];

%find input
u = TrajToInput(traj, physparams, vehicleparams);

%see what orbit we end in
addedtime = (traj.thist(end)+1):100:(traj.thist(end)+6000);
addedinput = zeros(2,length(addedtime));

%padded input
uadded = [u, addedinput];

%propagate our initial traj
xhistInit = PropTraj([traj.thist addedtime], uadded, initstate, physparams, vehicleparams);

% %magnitude of thrust at each time step
% umag = zeros(1,N);
% for ii = 1:N
%     umag(ii) = norm(u(:,ii));
% end
%     

figure
plot(xhistInit(1,:), xhistInit(2,:))
hold on
quiver(xhistInit(1,1:10:end), xhistInit(2,1:10:end), uadded(1,1:10:end), uadded(2,1:10:end))
axis equal
grid on

%find the cost of this u
uvector = reshape(u,2*(N-1),1);
tic
J = TrajCost_mex(uvector, traj.thist, initstate, physparams, vehicleparams, 0, rtarg);
toc
disp(J)

%form a function
funJ = @(u) TrajCost_mex(u, traj.thist, initstate, physparams, vehicleparams, 0, rtarg);

%derivative function
funJderiv = @(u) FD(u,funJ,1E-4);

% tic
% Jprime2 = funJderiv(uvector);
% toc

%evaluate derivative
% tic
% Jprime = TrajCostDeriv_mex(uvector, traj.thist, initstate, physparams, vehicleparams, 0, rtarg);
% toc

%set optimizer options
options.method = 'BFGS';
options.LS_initStride = .1;
options.LS_tol = 1E-3;
options.optTol = 1E-3;
options.maxIter = 100;
options.dispFreq = 1;

%Call optimizer
% [xHist, bigHist, JacCount, HesCount] = genOptimizer(funJ,funJderiv,uvector,options);
% 
% uFinal = xHist{end}(:,end);
% uFinal = reshape(uFinal,2,N-1);

matoptoptions = optimoptions('lsqnonlin','Display','iter-detailed');
uFinal = lsqnonlin(funJ, uvector,[],[], matoptoptions);
uFinal = reshape(uFinal,2,N-1);


%propagate final trajectory
xhist = PropTraj([traj.thist addedtime], [uFinal, addedinput], initstate, physparams, vehicleparams);

figure
plot(xhist(1,:), xhist(2,:))
hold on
quiver(xhist(1,1:10:end), xhist(2,1:10:end), uadded(1,1:10:end), uadded(2,1:10:end))
axis equal
grid on
