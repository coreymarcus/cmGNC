function [t_hist, x_hist] = PropTrajSegmentCoast(x0, t0, tf, physparams, vehicleparams)
%PropTrajSegment propagates a rocket trajectory from t0 to tf given state
%x0 assuming a constant control input along the trajectory
% INPUTS
%   x0 - initial rocket state
%   t0 - initial time
%   tf - final time
%   u - [2x1] control input
% 
% OUTPUTS
%   xf - final state

% create function for ode45
fun = @(t,x) RocketDynamicsCoast(t, x, physparams, vehicleparams);

%call ode45
options = odeset('AbsTol',1E-12,'RelTol',1E-12);
[t_hist, x_hist] = ode45(fun, [t0 tf], x0, options);

end

