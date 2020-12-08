function [ang] = EvalTimeTiltPoly(t, P)
%EvalTimeTiltPoly - Evaluates the time tilt polynomial 

%fraction
frac = t/P.tf;

%cases
if(frac < 0.25)
    ang = P.a(1)*t^5 + P.b(1)*t^4 + P.c(1)*t^3 + P.d(1)*t^2 + P.e(1)*t + P.f(1);
elseif( (frac >= 0.25) && (frac < 0.5) )
    ang = P.a(2)*t^5 + P.b(2)*t^4 + P.c(2)*t^3 + P.d(2)*t^2 + P.e(2)*t + P.f(2);
elseif( (frac >= 0.5) && (frac < 0.75) )
    ang = P.a(3)*t^5 + P.b(3)*t^4 + P.c(3)*t^3 + P.d(3)*t^2 + P.e(3)*t + P.f(3);    
else
    ang = P.a(4)*t^5 + P.b(4)*t^4 + P.c(4)*t^3 + P.d(4)*t^2 + P.e(4)*t + P.f(4);
end

end

