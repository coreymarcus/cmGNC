function dxdt = RocketDynamicsIGM(t,x,physparams,vehicleparams, chi_tilde, K1, PhiT, K2, T2, t0)
%RocketDynamics produces the change in state per unit of time of the rocket
% 
% INPUTS:
% t - scalar time, currently unused but necessary later
% x - state at time t, has the form of [position, velocity, mass]^T
% u - [2x1] control input, force of thrust in x and y directions
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

%caculate IGM pitch angle
chi = chi_tilde - K1 - PhiT + K2*(t - t0);

%vect along force
u_hat = [cos(chi); sin(chi)];

%force mag
g0 = abs(Grav(physparams.earthradius_m,physparams.earthgrav,1.0));
F = g0*vehicleparams.Isp*vehicleparams.m_dot;

%check for cutoff
if(T2 < 1)
    F = 0;
    disp("MECO!")
end

%force
u = F*u_hat;

% change in position
dxdt(1:2) = vel;

% find current altitude
h = norm(pos) - physparams.earthradius_m;

% atmosphereic density
rho = AirDensity(h, physparams.atmoheight_m);

% force of drag
Fdrag = Drag(vel, rho, vehicleparams.coeffdrag, vehicleparams.effectivearea);

% force of gravity
Fgrav = Grav(pos, physparams.earthgrav, m);

% acceleration of vehicle
dxdt(3:4) = (Fdrag + Fgrav + u)/m;

%change in mass
dxdt(5) = -1*vehicleparams.m_dot;


end

