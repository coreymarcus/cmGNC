function [vect] = HeightPolyToVect(P)
%PolyToVect Converts the time tilt polynomial into a vector for
%optimization

%resize
vect = reshape(P.coeffs(2:end,:),P.N*(P.deg-1),1);

end

