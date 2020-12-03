function J = TrajCost(u, thist, initstate, physparams, vehicleparams, etarg, atarg, lamb, p)
%TrajCost finds the cost of a given input u which must be minimized

%find N
N = length(u)/2;

%reshape the input
uarray = reshape(u,2,N);

%integrate the thrust
Jthrust = 0;
for ii = 1:N
    umag = norm(uarray(:,ii));
    dt = thist(ii+1) - thist(ii);
    Jthrust = Jthrust + umag*dt;
end

%now, propagate
xhist = PropTraj(thist, uarray, initstate, physparams, vehicleparams);

%evalutate constraints
psi = constraintEval(xhist(:,end), atarg, etarg, uarray, vehicleparams.vehiclethrust_N, physparams.earthgrav);

%total cost
J = Jthrust + lamb.'*psi + sum(p.*(psi.^2))/2;

end

