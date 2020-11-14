function rho = AirDensity(h)
%AirDensity produces the air density given altitude, currently using an
%extemely basic decaying exponential
% 
% INPUTS
% h = altitude above sea-level [m]

%approximate air density at sea level
rho_sea = 1.225; %kg/m^3
karm = 100000;
alpha = 6; %control decay rate

%calculate density
if(h >= karm) %karman line
    rho = 0;
else
    rho = rho_sea*exp(-alpha*h/karm);
end

end

