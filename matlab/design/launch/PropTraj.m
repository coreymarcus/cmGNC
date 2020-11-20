function xhist = PropTraj(t, u, x0, physparams, vehicleparams)
%PropTraj propagates the trajectory of a rocket given a time vector and
%control inputs
% 
% INPUTS
% t = [1xN] vector of times for evaluation
% u = [2x(N-1)] array of control inputs to be applied beginning at each time
%   and constantly until the next time
% x0 = [5x1] initial state
% physparams - physics parameters
% vehicleparams - vehicle parameters
% 
% OUTPUTS
% xhist = [5xN] time history

%extract local variables
N = length(t);

%initialize output
xhist = zeros(5,N);
xhist(:,1) = x0;

%loop
for ii = 1:(N-1)
    
    %times
    t0 = t(ii);
    tf = t(ii+1);
    
    %call segment propagator
    [~, xhistseg] = PropTrajSegment(xhist(:,ii),...
        t0, tf, u(:,ii), physparams, vehicleparams);
    
    %assign
    xhist(:,ii+1) = xhistseg(end,:)';
    
end


end

