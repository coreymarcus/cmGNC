clear
close all
clc

%final time
tf = 10;

%atmo height
h_atmo = 100000;

%Generate polynomial
P = GenInitPoly(tf);
deg = 3;
num_seg = 3;
Ph = GenInitHeightTiltPoly(h_atmo,num_seg,deg);

%test poly
t = 0:0.01:tf;
L = length(t);
poly_eval = zeros(L,1);
for ii = 1:L
    poly_eval(ii) = EvalTimeTiltPoly(t(ii), P);
end

%test height poly
h = 0:100:h_atmo;
L2 = length(h);
h_poly_eval = zeros(L2,1);
for ii = 1:L2
    h_poly_eval(ii) = EvalHeightTiltPoly(h(ii),Ph);
end

%plot
figure
subplot(2,1,1)
plot(t,poly_eval)
subplot(2,1,2)
plot(h,h_poly_eval);

%peturb polynomial
P.a(3) = .1;
P.b(1) = .5;
P = EnforceTimeTiltPolyConsts(P);

Ph.coeffs(2,1) = .2;
Ph.coeffs(3,2) = 5;
Ph = EnforceHeightTiltPolyConsts(Ph);

%evaluate peturbed
t = 0:0.01:tf;
L = length(t);
poly_eval = zeros(L,1);
for ii = 1:L
    poly_eval(ii) = EvalTimeTiltPoly(t(ii), P);
end

for ii = 1:L2
    h_poly_eval(ii) = EvalHeightTiltPoly(h(ii),Ph);
end

%plot
figure
subplot(2,1,1)
plot(t,poly_eval)
subplot(2,1,2)
plot(h,h_poly_eval)


%test d var creation
dvar = HeightPolyToVect(Ph);
PhCopy = VectToHeightPoly(dvar, h_atmo, num_seg, deg);