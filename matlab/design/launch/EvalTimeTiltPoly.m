function [ang] = EvalTimeTiltPoly(t, P)
%EvalTimeTiltPoly - Evaluates the time tilt polynomial 

%fraction
tFrac = t/P.tf;

%cases
if(tFrac < 0.25)
    ang = P.a(1)*tFrac^5 + P.b(1)*tFrac^4 + P.c(1)*tFrac^3 + P.d(1)*tFrac^2 + P.e(1)*tFrac + P.f(1);
elseif( (tFrac >= 0.25) && (tFrac < 0.5) )
    ang = P.a(2)*tFrac^5 + P.b(2)*tFrac^4 + P.c(2)*tFrac^3 + P.d(2)*tFrac^2 + P.e(2)*tFrac + P.f(2);
elseif( (tFrac >= 0.5) && (tFrac < 0.75) )
    ang = P.a(3)*tFrac^5 + P.b(3)*tFrac^4 + P.c(3)*tFrac^3 + P.d(3)*tFrac^2 + P.e(3)*tFrac + P.f(3);    
else
    ang = P.a(4)*tFrac^5 + P.b(4)*tFrac^4 + P.c(4)*tFrac^3 + P.d(4)*tFrac^2 + P.e(4)*tFrac + P.f(4);
end

end

