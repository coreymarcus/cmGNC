function [vect] = PolyToVect1stDeg(P)
%PolyToVect Converts the time tilt polynomial into a vector for
%optimization

%initalize
vect = zeros(4*1 + 1,1); %one for each free coeff plus final time

%populate
vect(1) = P.tf/(10^8); %rescale for conditioning
vect(2:5) = P.e;

end

