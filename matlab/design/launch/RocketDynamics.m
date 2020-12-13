function dxdt = RocketDynamics(t,x,P,physparams,vehicleparams)
%RocketDynamics produces the change in state per unit of time of the rocket
% 
% INPUTS:
% t - scalar time, currently unused but necessary later
% x - state at time t, has the form of [position, velocity, mass]^T
% P - time tilt polynomial
% physparams - physics paramter structure with the following fields
%   params.earthradius_m - radius of the earth
%   params.atmopheight_m - height of the atmosphere
%   params.earthmass_kg - mass of the earth
%   params.earthgrav - gravity parameter for earth
% vehicleparams - vehicle parameter structure with the following fields
%   params.coeffdrag - coefficient of drag
%   params.effectivearea - effective cross sectional area

% initialize output
dxdt = zeros(5,1);

% parse state
pos = x(1:2);
vel = x(3:4);
m = x(5);
e_int = x(6);

%get tilt
xi = EvalTimeTiltPoly(t,P);

%get current position angle
Phi = atan2(pos(2),pos(1)) - pi/2;

% force of gravity
Fgrav = Grav(pos, physparams.earthgrav, m);

% find current altitude
h = norm(pos) - physparams.earthradius_m;

% atmosphereic density
rho = AirDensity(h, physparams.atmoheight_m);

%tilt angle in inertial frame
xi_ecef = xi + Phi;

%get speed
spd = norm(vel);

% terminal speed PID with NaN protection
if (rho > 0)
    
    % term vel
    Vt = sqrt((2*norm(Fgrav))/(rho*vehicleparams.effectivearea*vehicleparams.coeffdrag));
    
    %PID thrust magnitude
    e = Vt - spd;
    u = vehicleparams.Kp*e + vehicleparams.Ki*e_int;
    
else
    %if we have broken atmosphere, T = Tmax
    u = vehicleparams.Tmax;
    e = 0;
end
    
%integrate controller error
dxdt(6) = e;

%saturate max/min thrust
if(u > vehicleparams.Tmax)
    u = vehicleparams.Tmax;
elseif(u < vehicleparams.Tmin)
    u = vehicleparams.Tmin;
end

%thrust
T = u*[cos(xi_ecef); sin(xi_ecef)];

% change in position
dxdt(1:2) = vel;

% force of drag
Fdrag = Drag(vel, rho, vehicleparams.coeffdrag, vehicleparams.effectivearea);

% acceleration of vehicle
dxdt(3:4) = (Fdrag + Fgrav + T)/m;

%change in mass
if(u > 0)
    dxdt(5) = -u/(vehicleparams.g0*vehicleparams.Isp);
end


end

