function g = Grav(r, mu, m2)
%grav produces a 2-D gravity force vector
%
% INPUTS:
% r - [2x1] position vector of secondary body (rocket) wrt center of
%   primary body
% mu - standard gravity parameter for primary body
% m2 - mass of secondary body (rocket)
% 
% OUTPUTS:
% g - [2x1] gravity force vector on the secondary body

% calculate magnitude of force
F = mu*m2/(r'*r);

% negative unit vector of position
v = -r/norm(r);

%output
g = v*F;

end

