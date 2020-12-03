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
vehicleparams.vehiclethrust_N = 15;
vehicleparams.coeffdrag = 0.75;
vehicleparams.effectivearea = 0.00001;

%% Optimization Parameters
N_nodes = 500; %number of discrete nodes for optimization
N_consts = 2 + N_nodes-1; %two final orbit constraints, N_nodes constraints for maximum thrust
max_major_iter = 100;
max_minor_iter = 1000;
epsMajor = 1E-5;

useMatlabSolver = true; %mine or matlabs
matoptions = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'Display','iter-detailed','FiniteDifferenceStepSize',1E-3,...
    'MaxIterations',100,'StepTolerance',1E-8,'FiniteDifferenceType','central');

%BFGS options
options.LS_initStride = .0001;
options.method = 'BFGS';
options.LS_tol = 1E-3;
options.optTol = 1E-4;
options.maxIter = 1000;
options.dispFreq = 1;

%% Other Setup
addpath('C:\Users\corey\Documents\GitHub\matlabScripts')
addpath('C:\Users\corey\Documents\GitHub\OST\Project')

%% Main

% Initial conditions
rtarg = physparams.earthradius_m + 2.5*physparams.atmoheight_m;
x0 = [0 physparams.earthradius_m]';
initstate = [x0; 0; 0; vehicleparams.vehiclemass_kg]; %initial vehicle state

%get a best guess on the initial trajectory
traj = GenerateInitialTrajectory(N_nodes, rtarg, x0, physparams, vehicleparams);

%find input
u_initial = TrajToInput(traj, physparams, vehicleparams);

%reshape initial input into decision variables
dvar = reshape(u_initial,2*(N_nodes-1),1);

%initialize optimization parameters
lamb = zeros(N_consts,1);
p = ones(N_consts,1);
majorIter = 1;
maxMajorIter = 100;
majorConv = false;
majorConvOnce = false;

%form functions
fun = @(u) TrajCost_mex(u, traj.thist, initstate, physparams, vehicleparams, 0, rtarg, lamb, p);
fun_jac = @(u) FD(u,fun,1E-1);

while ~majorConv
    
    %extract the latest force input guess
    dvar_iter = dvar(:,end);
    
    %call solver
    if useMatlabSolver
        dvar_iter_opt = lsqnonlin(fun, dvar_iter, [], [], matoptions);
        dvar = [dvar, dvar_iter_opt];
        
    else
        
        %call my solver
        [dVarHistCell, ~, ~, ~] = genOptimizer(fun,fun_jac,dvar_iter,options);
    
        %add latest decision variable
        dvar = [dvar, dVarHistCell{end}(:,end)];
    
    end
    
    %propagate current best input
    u_iter = reshape(dvar(:,end), 2,N_nodes-1);
    xhist = PropTraj(traj.thist, u_iter, initstate, physparams, vehicleparams);
    
    %evaluate constraints
    psi = constraintEval(xhist(:,end), rtarg, 0, u_iter, vehicleparams.vehiclethrust_N, physparams.earthgrav);
    
    %convergence check logic
    convCheck = zeros(length(lamb),1);
    
    %update lagrangian parameters
    for ii = 1:length(lamb)
        if abs(psi(ii)) > epsMajor
            lamb(ii) = lamb(ii) + p(ii)*psi(ii);
            p(ii) = 2*p(ii);
        else
            convCheck(ii) = 1;
        end
    end
    
    %update functions
    fun = @(u) TrajCost_mex(u, traj.thist, initstate, physparams, vehicleparams, 0, rtarg, lamb, p);
    fun_jac = @(u) FD(u,fun,1E-4);
    
    %loop managment
    disp('$$$$$$$$$$$$$$$$$ MAJOR ITERATION $$$$$$$$$$$$$$$$$')
    fprintf('Iter: %i  Constraints Satisfied: %i / %i \n \n',majorIter,sum(convCheck),length(convCheck))
    
    
    if(min(convCheck) > 0 && majorConvOnce)
        disp('Converged Once!')
        majorConv = true;
    elseif(min(convCheck) > 0 && ~majorConvOnce)
        disp('Converged Once!')
        majorConvOnce = true;
    else
        majorConvOnce = false;
    end
    
    if majorIter > maxMajorIter
        disp('Max Iterations Reached.')
        break;
    end
    
    majorIter = majorIter + 1;
    
    
end
