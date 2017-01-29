function [ Q ] = Qeci2peri( omeg, inc, RAAN )

R31=[cosd(omeg), sind(omeg), 0;
     -sind(omeg), cosd(omeg), 0;
     0, 0 , 1];
R1=[1 , 0, 0;
    0, cosd(inc), sind(inc);
    0, -sind(inc), cosd(inc)];
R32=[cosd(RAAN), sind(RAAN), 0;
     -sind(RAAN), cosd(RAAN), 0;
     0, 0 , 1];

Q=R31*R1*R32;


end

