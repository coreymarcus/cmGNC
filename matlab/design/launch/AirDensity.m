function rho = AirDensity(h, atmoheight_m)
%AirDensity produces the air density given altitude, currently using an
%extemely basic decaying exponential
% 
% INPUTS
% h = altitude above sea-level [m]
% atmoheight = height of the atmosphere [m]

%approximate air density at sea level
rho_sea = 1.225; %kg/m^3
alpha = 6; %control decay rate

%calculate density
if(h >= atmoheight_m) %karman line
    rho = 0;
elseif(h <= 0)
    rho = rho_sea;
else
    rho = rho_sea*exp(-alpha*h/atmoheight_m);
end

end

