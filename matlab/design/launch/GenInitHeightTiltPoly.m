function [P] = GenInitHeightTiltPoly(h_atmo, N, deg)
%GenInitPoly Creates the initial height tilt polynomial
% coeffs => 1 row for each degree, one col for each segment

%initialize
P.hf = h_atmo;
P.N = N;
P.deg = deg;
P.coeffs = zeros(deg,N);

%init tilt
P.coeffs(1,1) = pi/2;

%final tilt
tiltf = 0*pi/2;

%total slope
m = (tiltf - P.coeffs(1,1));

%linear slope components of polys
P.coeffs(2,:) = m*ones(N,1);

%update constant component
if(N > 1)
    for ii = 2:N
        P.coeffs(1,ii) = (P.coeffs(2,ii-1) - P.coeffs(2,ii))*(ii/N) + P.coeffs(1,ii-1);
    end
end

end

