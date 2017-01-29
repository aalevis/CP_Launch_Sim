function  disp_coes( h, i, RAAN, ecc, omeg, theta )
mu = 398600;
a = h^2/(mu*(1-ecc^2));
disp(['Semi-Major = ' num2str(a) ' km'])
disp(['ecc = ' num2str(ecc)])
disp(['Inc = ' num2str(i)])
disp(['RAAN = ' num2str(RAAN)])
disp(['Arg of Peri = ' num2str(omeg)])
disp(['TA = ' num2str(theta)])


end

