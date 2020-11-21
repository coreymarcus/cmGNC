function u = TrajToInput(traj,physparams, vehicleparams)
%TrajToInput creates an input u based on the input trajectory, accounts for
% vehicle mass and air resistance and gravity

%initialize input
N = size(traj.pos,2);
u = zeros(2,N-1);

for ii = 1:(N-1)
    
    %state
    pos0 = traj.pos(:,ii);
    posf = traj.pos(:,ii+1);
    vel0 = traj.vel(:,ii);
    velf = traj.vel(:,ii+1);
    m = vehicleparams.vehiclemass_kg;
    posavg = (pos0 + posf)/2;
    velavg = (vel0 + velf)/2;
    accel = traj.accel(:,ii);
    
    %approximate gravity
    Fg = Grav(posavg, physparams.earthgrav, m);
    
    %approximate drag
    rho = AirDensity(norm(posavg) - physparams.earthradius_m, physparams.atmoheight_m);
    Fd = Drag(velavg, rho, vehicleparams.coeffdrag, vehicleparams.effectivearea);
    
    %find force input required
    u(:,ii) = m*accel - Fg - Fd;
    
end
end

