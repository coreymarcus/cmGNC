function [t_hist, x_hist, new_err_int] = PropTrajSegment(x0, t0, tf, P, physparams, vehicleparams, err_int)
%PropTrajSegment propagates a rocket trajectory from t0 to tf
% used for atmospheric flight!
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
options = odeset('AbsTol',1E-10,'RelTol',1E-10);
[t_hist, x_hist] = ode45(fun, [t0 tf], [x0; err_int], options);

%trim output
new_err_int = x_hist(end,6);
x_hist = x_hist(:,1:5);

end

