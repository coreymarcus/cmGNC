function [P] = VectToPoly1stDeg(vect)
%VectToPoly Converts decision vector to time tilt polynomial

%initialize
P.tf = vect(1)*10^8; %rescale for conditioning
P.a = zeros(4,1);
P.b = zeros(4,1);
P.c = zeros(4,1);
P.d = zeros(4,1);
P.e = vect(2:5);
P.f = zeros(4,1);

%enforce constraints
P = EnforceTimeTiltPolyConsts(P);

end

