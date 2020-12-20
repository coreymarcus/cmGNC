function [P] = VectToHeightPoly(vect,h_atmo, Nseg, Ndeg)
%VectToPoly Converts decision vector to time tilt polynomial

%append initial tilt
[n, m] = size(vect);
if(n > m)
    vect = vect'; %make sure it is a row
end
P.coeffs = zeros(Ndeg,Nseg);
P.coeffs(1,1) = pi/2; %initial tilt
P.coeffs(2:end,:) = reshape(vect,Ndeg-1,Nseg);

%other parameters
P.N = Nseg;
P.hf = h_atmo;
P.deg = Ndeg;

%enforce constraints
P = EnforceHeightTiltPolyConsts(P);

end

