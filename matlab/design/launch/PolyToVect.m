function [vect] = PolyToVect(P)
%PolyToVect Converts the time tilt polynomial into a vector for
%optimization

%initalize
vect = zeros(4*5 + 1,1); %one for each free coeff plus final time

%populate
vect(1) = P.tf;
vect(2:5) = P.a;
vect(6:9) = P.b;
vect(10:13) = P.c;
vect(14:17) = P.d;
vect(18:21) = P.e;

end

