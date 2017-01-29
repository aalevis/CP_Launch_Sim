function [dV_K] = potential_dv(alt)

mu_e = 398600;
r_e = 6378;
K_g = mu_e/r_e;

K_alt = mu_e./(r_e + alt); % kJ/kg
dE_sp =  K_g - K_alt;  % kJ/kg
dV_K = sqrt(2*dE_sp);

end