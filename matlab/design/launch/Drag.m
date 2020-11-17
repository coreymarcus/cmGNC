function F = Drag(v, rho, Cd, A)
%drag produces a 2-D drag force vector
%
% INPUTS:
% v - [2x1] velocity vector of the vehicle wrt the air
% rho - the air density
% Cd - the coefficient of drag
% A - the cross sectional area of the vehicle
%  
% OUTPUTS:
% F - [2x1] the force vector of drag

%calculate force
Fmag = 0.5*rho*(v'*v)*Cd*A;

%calculate anti-parallel velocity vector
nv = norm(v);
if(nv == 0)
    F = [0 0]'; %guard against divide by zero
else
    
    %anti-parallel vector
    vect = -v/norm(v);

    %output
    F = Fmag*vect;
end

end

