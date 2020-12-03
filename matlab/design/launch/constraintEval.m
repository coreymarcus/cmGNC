function [psi] = constraintEval(xf, a_targ, e_targ, u_array, umax, mu)
%contstraintEval evaluates all equality and inequality constraints

%final state in 3D
posf = [xf(1:2); 0];
velf = [xf(3:4); 0];

%normalized r and v
nr = norm(posf);
nv = norm(velf);

%angular momentum vector
h = cross(posf,velf);

%semi-major axis [m]
a = -mu/(nv^2-2*mu/nr);

%eccentricity vector
evect = (cross(velf,h)-mu*posf/nr)/mu;

%ecentricity
e = norm(evect);

%find places where umag > umax
N_u = size(u_array,2);
u_constraint_cost = zeros(N_u,1);

for ii = 1:N_u
    umag = norm(u_array(:,ii));
    if(umag > umax)
        u_constraint_cost(ii) = umag - umax;
    end
end

%constraint evaluation
psi = [a - a_targ;
    e - e_targ;
    u_constraint_cost];

end

