function [chi_tilde, K1, PhiT, K2, T2] = IGM(x, g0, R0, RT, VT, m12, Vex2, m2dot, xiT_dot, etaT_dot)
%IGM Iterative Guidance Mode - an Implementation of the iterative guidance
%mode employed on the Saturn V. See "An Iterative Guidance Scheme for
%Ascent to Orbit" Smith and Deaton, 1963, NASA Technical Report

%% Setup
%extract state variables
x1 = x(1);
y1 = x(2);
vx1 = x(3);
vy1 = x(4);

%% Compute V1, g*, Phi1

%current velocity
V1 = sqrt(vx1^2 + vy1^2);

%current radius
R1 = sqrt(x1^2 + y1^2);

%current angle
Phi1 = atan(x1/y1);

%current gravity
g1 = g0*(R0/R1)^2;

%injection point gravity
gT = g0*(R0/RT)^2;

%average gravity
g_star = 0.5*(g1 + gT);

%% Compute Estimated time to go in second stage, T2star

%estimated time until complete burn-off of current stage
tau2 = m12/m2dot;

%estimated time remaining in second stage
T2_star = tau2*(1 - exp((V1 - VT)/Vex2));

%% Compute Phi12, PhiT, and Phi_star

%compute this variable, not sure what it means
Phi12 = (V1*T2_star + Vex2*(T2_star - (tau2 - T2_star)*log(tau2/(tau2 - T2_star))))/RT;

%estimated point for cutoff
PhiT = Phi1 + Phi12;

%Average angle remaining?
Phi_star = 0.5*Phi12;

%% Compute Injection Coordinates

%assume T2prime = T2_star (may not be good for 2-stage vehicle)
T2prime = T2_star;

%we need this matrix
tempmat = [cos(PhiT), -sin(PhiT);
    sin(PhiT), cos(PhiT)];

%current position in injection coords
tempvec = tempmat*[x1; y1];
xi1 = tempvec(1);
eta1 = tempvec(2);

%current velocity in injection coords
tempvec = tempmat*[vx1; vy1];
etadot1 = tempvec(1);
xidot1 = tempvec(2); %Intetional reversal according to paper!

%required change in velocity
deltaxidot_star = xiT_dot - xidot1 - g_star*T2prime*sin(Phi_star);
deltaetadot_star = etaT_dot - etadot1 + g_star*T2prime*cos(Phi_star);

%% Compute Delta T2 and Time Remaining

%compute a bunch of parameters without physical intuition
lambda = g_star*(deltaxidot_star*cos(Phi_star) - deltaetadot_star*sin(Phi_star));
deltaVstar2 = deltaetadot_star^2 + deltaxidot_star^2;
L = Vex2*log(tau2/(tau2 - T2prime));
K = Vex2/(tau2 - T2prime);
a = K^2 - g_star^2;
b = lambda - L*K;
c = deltaVstar2 - L^2;
deltaT2 = (b + sqrt(b^2 + a*c))/a;


%time remaining
T2 = T2prime + deltaT2;

%% Compute chi_tilde
chi_tilde = atan((deltaetadot_star + g_star*deltaT2*cos(Phi_star))/(deltaxidot_star - g_star*deltaT2*sin(Phi_star)));

%% Compute K1 and K2

%assume etaT = RT
etaT = RT;

%need a bunch of intermediate calcs
A1 = Vex2*log(tau2/(tau2 - T2));
B1 = Vex2*(tau2*log(tau2/(tau2-T2)) - T2);
A2 = cos(chi_tilde)*Vex2*(T2 - (tau2 - T2)*log(tau2/(tau2 - T2)));
B2 = cos(chi_tilde)*(-Vex2)*(0.5*T2^2 - tau2*(T2 - (tau2 - T2)*log(tau2/(tau2 - T2))));
C2 = eta1 - etaT + etadot1*T2 - 0.5*g_star*T2^2*cos(Phi_star) + sin(chi_tilde)*Vex2*(T2 - (tau2-T2)*log(tau2/(tau2-T2)));

K1 = B1*C2/(A2*B1 - A1*B2);
K2 = A1*K1/B1;


end

