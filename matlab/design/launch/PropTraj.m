function xhist = PropTraj(t, P, x0, physparams, vehicleparams)
%PropTraj propagates the trajectory of a rocket given a time vector and
%control inputs
%
% INPUTS
% t = [1xN] vector of times for evaluation
% P = time tilt polynomial
% x0 = [5x1] initial state
% physparams - physics parameters
% vehicleparams - vehicle parameters
%
% OUTPUTS
% xhist = [5xN] time history

%extract local variables
N = length(t);

%IGM Inputs
R0 = physparams.earthradius_m;
RT = R0 + 2.5*physparams.atmoheight_m;
g0 = -1*Grav(R0,physparams.earthgrav,1.0);
VT = sqrt(physparams.earthgrav/RT);
m12 = vehicleparams.vehiclemass_kg;
Vex2 = vehicleparams.Isp*g0;
m2dot = vehicleparams.m_dot_max;
xiT_dot = VT;
etaT_dot = 0;

%initialize output
xhist = zeros(5,N);
xhist(:,1) = x0;

%PID controller error
err_int = 0;

%loop
MECOflag = false;
dispflag = true;
althist = zeros(N,1);
for ii = 1:(N-1)
    
    %times
    t0 = t(ii);
    tf = t(ii+1);
    
    %get altitude, check if we have left the atmo
    alt = norm(xhist(1:2,ii)) - physparams.earthradius_m;
    althist(ii) = alt;
    plot(t,althist)
    drawnow
    if(alt < 0)
        disp("Crash!")
        break
    elseif(xhist(5,ii) < 0)
        disp("Negative Mass!")
        break
    elseif((alt >= 0) && (alt < physparams.atmoheight_m) && ~MECOflag)
        
        %call segment propagator
        [~, xhistseg, err_int] = PropTrajSegment(xhist(:,ii),...
            t0, tf, P, physparams, vehicleparams, err_int);
        
    elseif(~MECOflag) %use IGM
        
        %update mass
        m12 = xhist(5,ii);
        
        %get params
        [chi_tilde, K1, PhiT, K2, T2] = IGM(xhist(1:4,ii),...
            g0, R0, RT, VT, m12, Vex2, m2dot, xiT_dot, etaT_dot);
        
        if(dispflag)
            dispflag = false;
            disp("Exiting Atmosphere...")
        end
        
        if(T2 < vehicleparams.cycletime_sec || MECOflag) %defined as 10 for good performance in IGM
            disp("MECO!")
            MECOflag = true;
            
            %call segment propagator
            [~, xhistseg] = PropTrajSegmentCoast(xhist(:,ii),t0, tf,...
                physparams, vehicleparams);
        else
            %call segment propagator
            [~, xhistseg] = PropTrajSegmentIGM(xhist(:,ii),t0, tf,...
                physparams, vehicleparams,chi_tilde, K1, PhiT, K2, T2);
        end
        
    else %MECO achieved, coast...
        
        %call segment propagator
        [~, xhistseg] = PropTrajSegmentCoast(xhist(:,ii),t0, tf,...
            physparams, vehicleparams);
        
        
    end
    
    
    %assign
    xhist(:,ii+1) = xhistseg(end,:)';
    
end


end

