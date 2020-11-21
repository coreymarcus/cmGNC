function [deriv] = TrajCostDeriv(uvector, thist, initstate, physparams, vehicleparams, atarg, rtarg);
%TrajCostDeriv finds derivative of cost

%form a function
funJ = @(u) TrajCost(u, thist, initstate, physparams, vehicleparams, atarg, rtarg);

%evaluate derivative
deriv = FD(uvector,funJ,1E-4);
end

