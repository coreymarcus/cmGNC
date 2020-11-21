function [traj] = GenerateInitialTrajectory(N, r, x0, physparams, vehicleparams)
%GenerateInitialTrajectory creates a first guess for control inputs
% 
% INPUTS
% N = number of nodes
% r = target circular orbit radius
% x0 = [2x1] initial position
%
% OUTPUTS
% traj = trajectory output structure with following fields
%   traj.pos = [2xN] position history
%   traj.vel = [2xN] velocity history
%   traj.accel = [2xN] acceleration history
%   traj.thist = [1xN] time history at each node

% General idea: fit ascent arc to ellipse in polar coordinates, choose
% velocity along the arc such that we hit circular velocity at the end

% find our initial angle
theta1 = atan2(x0(2),x0(1));

% arbitrarily designate final angle
theta2 = theta1 + pi/4;

% starting point
r1 = norm(x0);

% final radius
r2 = r;

%axis lengths
a = theta2 - theta1;
b = r2 - r1;

%sample
% theta_samp = linspace(theta1,theta2,L);
theta_samp = logspace(log10(theta1),log10(theta2),N);
r_samp = zeros(N,1);
for ii = 1:N
    r_samp(ii) = r1 + b*sqrt(1 - (theta_samp(ii) - theta2)^2/a^2);
end


% figure
% subplot(2,1,1)
% plot(theta_samp,r_samp)
% subplot(2,1,2)
% plot(theta_samp,1:L)

%find the final tangential velocity
safefactor = 1;
vcircle = safefactor*sqrt(physparams.earthgrav/r2);

%convert to cartesian coordinates
x_targ = r_samp'.*cos(theta_samp);
y_targ = r_samp'.*sin(theta_samp);

%find the approximate arc length
pathtrack = zeros(1,N);
pathlen = 0;
for ii = 1:(N-1)
    pathlen = pathlen + sqrt((x_targ(ii+1) - x_targ(ii))^2 + (y_targ(ii+1) - y_targ(ii))^2);
    pathtrack(ii+1) = pathlen;
end

%assume we want our tangential velocity to be the circular velcoity at the
%end of the arc
tfinal = 2*pathlen/vcircle;
acceltang = vcircle/tfinal;

%time of arrival at each node
timetrack = sqrt(2*pathtrack/acceltang);

%we can now create velocity targets at each node
x_vel_targ = zeros(1,N);
y_vel_targ = zeros(1,N);
for ii = 2:N
    %tangential velocity at this node
    v = acceltang*sqrt(2*pathtrack(ii)/acceltang);
    
    if(ii < N)
        %tan behind and in front
        tanback = [(x_targ(ii) - x_targ(ii-1));
            (y_targ(ii) - y_targ(ii-1))];
        tanfront = [(x_targ(ii+1) - x_targ(ii));
            (y_targ(ii+1) - y_targ(ii))];
        
        %average tangent
        tanavg = (tanback + tanfront)/2;
        
        %normalize
        tanavg = tanavg/norm(tanavg);
    else
        %final point
        tanavg = [-y_targ(ii); x_targ(ii)];
        tanavg = tanavg/norm(tanavg);
    end
    
    %velocity target
    x_vel_targ(ii) = v*tanavg(1);
    y_vel_targ(ii) = v*tanavg(2);   
    
end

%finally, we must find the acceleration target for each step
x_accel_targ = zeros(1,N);
y_accel_targ = zeros(1,N);
for ii = 1:(N-1)
    
    %change in variables
    dt = timetrack(ii+1) - timetrack(ii);
    dvelx = x_vel_targ(ii+1) - x_vel_targ(ii);
    dvely = y_vel_targ(ii+1) - y_vel_targ(ii);
    
    %calculate required acceleration to meet next velocity
    x_accel_targ(ii) = dvelx/dt;
    y_accel_targ(ii) = dvely/dt;
end

% %parameters for drawing
% thetadraw = linspace(pi/2,pi,N);
% 
% %drawing
% x_r1 = r1*cos(thetadraw);
% y_r1 = r1*sin(thetadraw);
% x_r2 = r2*cos(thetadraw);
% y_r2 = r2*sin(thetadraw);
% 
% figure
% plot(x_targ,y_targ,...
%     x_r1,y_r1,...
%     x_r2,y_r2)
% hold on
% % quiver(x_targ,y_targ,x_vel_targ,y_vel_targ)
% quiver(x_targ,y_targ,x_accel_targ,y_accel_targ)
% axis equal


%Outputs
traj.pos = [x_targ; y_targ];
traj.vel = [x_vel_targ; y_vel_targ];
traj.accel = [x_accel_targ; y_accel_targ];
traj.thist = timetrack;


end

