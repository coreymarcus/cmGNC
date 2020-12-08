function [P] = EnforceTimeTiltPolyConsts(P)
%EnforceTimeTiltPolyConsts enforces polynomial continuity

%initial angle
P.f(1) = pi/2;

%loop
for ii = 2:4
    P.f(ii) = (P.a(ii-1) - P.a(ii))*((ii-1)*P.tf/4)^5 + ...
        (P.b(ii-1) - P.b(ii))*((ii-1)*P.tf/4)^4 + ...
        (P.c(ii-1) - P.c(ii))*((ii-1)*P.tf/4)^3 + ...
        (P.d(ii-1) - P.d(ii))*((ii-1)*P.tf/4)^2 + ...
        (P.e(ii-1) - P.e(ii))*((ii-1)*P.tf/4) + P.f(ii-1);
end

end

