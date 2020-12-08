function [P] = GenInitPoly(tf)
%GenInitPoly Creates the initial time tilt polynomial

%initialize
P.tf = tf;
P.a = zeros(4,1);
P.b = zeros(4,1);
P.c = zeros(4,1);
P.d = zeros(4,1);
P.e = zeros(4,1);
P.f = zeros(4,1);

%init tilt
P.f(1) = pi/2;

%total slope
m = -P.f(1)/(2*tf);

%linear slope components of polys
P.e = m*ones(4,1);

for ii = 2:4
    P.f(ii) = (P.e(ii-1) - P.e(ii))*(ii/4)*tf + P.f(ii-1);
end

end

