function [t_hist, x_hist] = PropTrajSegment(x0, t0, tf, P, physparams, vehicleparams)
%PropTrajSegment propagates a rocket trajectory from t0 to tf given state
%x0 assuming a constant control input along the trajectory
% INPUTS
%   x0 - initial rocket state
%   t0 - initial time
%   tf - final time
%   P - time tilt polynomial
% 
% OUTPUTS
%   xf - final state

% create function for ode45
fun = @(t,x) RocketDynamics(t, x, P, physparams, vehicleparams);

%call ode45
[t_hist, x_hist] = ode45(fun, [t0 tf], x0);

end

