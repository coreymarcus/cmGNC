clear
close all
clc

%final time
tf = 10;

%Generate polynomial
P = GenInitPoly(tf);

%test poly
t = 0:0.01:tf;
L = length(t);
poly_eval = zeros(L,1);
for ii = 1:L
    poly_eval(ii) = EvalTimeTiltPoly(t(ii), P);
end

%plot
figure
plot(t,poly_eval)

%peturb polynomial
P.a(3) = .1;
P.b(1) = .5;
P = EnforceTimeTiltPolyConsts(P);

%evaluate peturbed
t = 0:0.01:tf;
L = length(t);
poly_eval = zeros(L,1);
for ii = 1:L
    poly_eval(ii) = EvalTimeTiltPoly(t(ii), P);
end

%plot
figure
plot(t,poly_eval)