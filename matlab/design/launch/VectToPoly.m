function [P] = VectToPoly(vect)
%VectToPoly Converts decision vector to time tilt polynomial

%initialize
P.tf = vect(1);
P.a = vect(2:5);
P.b = vect(6:9);
P.c = vect(10:13);
P.d = vect(14:17);
P.e = vect(18:21);
P.f = zeros(4,1);

%enforce constraints
P = EnforceTimeTiltPolyConsts(P);

end

