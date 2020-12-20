function [ang] = EvalHeightTiltPoly(h, P)
%EvalHeightTiltPoly - Evaluates the height tilt polynomial

%detirmine which segment to use
seg = floor(h*P.N/P.hf) + 1;

%point to evaluate (normalized such that evaluation variable is 1 at
%h_atmo)
eval_pt = h/P.hf;

%polynomial degree
deg = P.deg;

%saturate segment
if(seg > P.N)
    seg = P.N;
elseif(seg < 1)
    seg = 1;
end

%evaluate
ang = 0;
for ii = 1:deg
    ang = ang + P.coeffs(ii,seg)*eval_pt^(ii-1);
end

end

