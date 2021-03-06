function J = TrajCost(u, thist, initstate, physparams, vehicleparams)
%TrajCost finds the cost of a given input time-tilt poly which must be
%minimized

%convert the input to a poly
if(vehicleparams.useheightpoly)
    poly = VectToHeightPoly(u, physparams.atmoheight_m, vehicleparams.PolySeg, vehicleparams.PolyDeg);
else
    poly = VectToPoly1stDeg(u);
end

%get initial mass
m1 = initstate(5);

%propagate trajectory
xhist = PropTraj(thist, poly, initstate, physparams, vehicleparams);

%get final mass
m2 = xhist(5,end);

%cost is fuel consumption
J = m1 - m2;


end

