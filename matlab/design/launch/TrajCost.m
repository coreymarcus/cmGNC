function J = TrajCost(u, thist, initstate, physparams, vehicleparams, etarg, atarg)
%TrajCost finds the cost of a given input u which must be minimized

%find N
N = length(u)/2;

%reshape the input
uarray = reshape(u,2,N);

%mu
mu = physparams.earthgrav;

%integrate the thrust
Jthrust = 0;
for ii = 1:N
    umag = norm(uarray(:,ii));
    dt = thist(ii+1) - thist(ii);
    Jthrust = Jthrust + umag*dt;
end

%now, propagate
xhist = PropTraj(thist, uarray, initstate, physparams, vehicleparams);

%final state in 3D
posf = [xhist(1:2,end); 0];
velf = [xhist(3:4,end); 0];

%normalized r and v
nr = norm(posf);
nv = norm(velf);

%angular momentum vector
h = cross(posf,velf);

%semi-major axis [m]
a = -mu/(nv^2-2*mu/nr);

%eccentricity vector
e = (cross(velf,h)-mu*posf/nr)/mu;

%ecentricity
ne = norm(e);

%OE costs
Ja = 0.0000001*(a - atarg)^2;
Je = 1000*(ne - etarg)^2;

%total cost
J = Jthrust + Ja + Je;

end

