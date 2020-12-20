function [P] = EnforceHeightTiltPolyConsts(P)
%EnforceHeightTiltPolyConsts enforces polynomial continuity

%initial angle
P.coeffs(1,1) = pi/2;

%number of segments
N = P.N;

%degree
deg = P.deg;

%loop
if(N > 1)
    for ii = 2:N
        
        %initialize sum term
        sum_term = P.coeffs(1,ii-1);
        if(deg > 1)
            for jj = 2:deg
                sum_term = sum_term + (P.coeffs(jj,ii-1) - P.coeffs(jj,ii))*...
                    ((ii - 1)/N)^(jj-1);
            end
        end
        
        %assign
        P.coeffs(1,ii) = sum_term;
    end
end

end

