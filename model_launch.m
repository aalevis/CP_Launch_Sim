function [ t1,f1 ] = model_launch( fid,external )
%model_launch All inputs in form 1xn matrix where n = # stages
%   m0     = gross mass                 [kg]
%   mp     = propellant mass            [kg]
%   Thrust = thrust of each stage        [N]
%   Isp    = specific impulse            [s]
%   m_frac = mass fraction of each stage [m_prop/(m_stages)]
%   diam   = diameter of each stage      [m]
%   t_burn = burn time of each stage     [s]

deg = pi/180;       % ...Convert degrees to radians
g0 = 9.81;          % ...Sea-level acceleration of gravity (m/s^2)
Re = 6378e3;        % ...Radius of the earth (m)
hscale = 7.5e3;     % ...Density scale height (m)
rho0 = 1.225;       % ...Sea level density of atmosphere (kg/m^3)
CD = 0.5;           % ...Drag coefficient (assumed constant)
hturn = 300;        % ...Height at which pitchover begins (m)
t0 = 0;             % ...Initial time for the numerical integration

% Load Roacket info
data = load(fid)     ;
m0     = data(1,:)   ; %[kg] gross mass
mp     = data(2,:)   ; %[kg] propellant mass
Thrust = data(3,:)   ; %[N]
Isp    = data(4,:)   ; %[s]
diam   = data(5,:)   ; %[m]
t_burn = data(6,:)   ; %[s]
for i = 1:length(m0)
    m_frac(i) = mp(i)/sum(m0(i:end)) ;
end

switch external
    case 'no'
        m0     = data(1,:)   ; %[kg] gross mass
        mp     = data(2,:)   ; %[kg] propellant mass
        Thrust = data(3,:)   ; %[N]
        Isp    = data(4,:)   ; %[s]
        diam   = data(5,:)   ; %[m]
        t_burn = data(6,:)   ; %[s]      
    case 'yes'
        
end

% ...Initial conditions:
v0 = 0;             % ...Initial velocity (m/s)
gamma0 = 87.85*deg; % ...Initial flight path angle (rad)
x0 = 0;             % ...Initial downrange distance (km)
h0 = 0;             % ...Initial altitude (km)
vD0 = 0;            % ...Initial value of velocity loss due to drag (m/s)
vG0 = 0;            % ...Initial value of velocity loss due to gravity (m/s)

%...Initial conditions vector:
f0 = [v0; gamma0; x0; h0; vD0; vG0];
options = odeset('Abstol', 1e-8, 'Reltol', 1e-8);

t_b = 0 ;
l = 0 ;
for i = 1:length(m_frac)
    A(i) = pi/4*(diam(i))^2;    % ...Frontal area (m^2)
    n(i) = 1/(1-m_frac(i));     % ...Mass ratio (mf1/mo)
    mfinal(i) = m0(i)/n(i);     % ...Burnout mass (kg)
    m_dot(i) = Thrust(i)/Isp(i)/g0;   % ...Propellant mass flow rate (kg/s) Assuming optimal expansion throughout flight
    mprop(i) = m0(i) - mfinal(i);     % ...Propellant mass (kg)
    mdot(i) = mprop(i)/t_burn(i);     % ...Burn time (s)
    tspan = [t0,t_burn(i)];           % ...Range of integration
    
    t_b = t_b + t_burn(i) ;
    [t, f] = ode45(@rocketrates, tspan, f0, options, t_burn(i), sum(m0(i:end)), m_dot(i),...
        g0, Re, Thrust(i), A(i), CD, hturn);
    
    l = length(t) + l ;
    if i == 1
        t1(1:l,1) = t ;
        f1(1:l,1:6) = f ;
    else
        t1 = [t1; t + ones(length(t),1)*t1(end)] ;
        f1(1:l,1:6) = [f1; f] ;
    end
    
    f0 = f1(end,:) ;
end


end