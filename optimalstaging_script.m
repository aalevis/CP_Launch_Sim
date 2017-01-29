%% Optimal Staging
close all
clear all
clc
format compact

%% Givens
% Going into elliptical orbit
mu = 398600; % gravitational constant
alt = [567 625]; % burnout altitude [imaging comms]
ecc = .2369; % eccentricity

for j = 1:length(alt)
rp(j) = alt(j)+6378; % perigee radius [imaging comms]
ra(j) = rp(j)*(1+ecc)/(1-ecc); % apogee radius [imaging comms]
a(j) = ((rp(j)+ra(j)))/2; % semimajor axis [imaging comms]
end

% Calculate potential energy
for j = 1:length(alt)
potential(j) = potential_dv(alt(j)); 
end

% payload mass (kg)
m_pl = 30:1:500;
% number of stages (1 - 5)
N = 2; 
% Specific Impulse of each stage (seconds)
isp = [265 305];
% Structure Ratios of each stage 
e = [.2 .1];
% Burnout Velocity (km/s)
for j = 1:length(alt)
v(j) = sqrt((-mu/(2*a(j)) + mu/(rp(j)))) + potential(j); % [imaging comms]
% v(j) = sqrt(mu/(6378+alt(j))); 
end
% Guess for newton's iteration (.1 - 1)
eta_guess = .5;

%% Use OptStaging.m
for j = 1:length(v)
for i = 1:length(m_pl)
[eta(i,j),MF(i,:,j),m_step(i,:,j),m_o(i,j),m_S(i,:,j),m_P(i,:,j),lam_total(i,j),lambda(i,:,j),check(i,:,j)] = OptStaging(isp,e,v(j),N,m_pl(i),eta_guess);
end
end
% Optimal Staging for a tandem stacked LV
% From Curtis chapter 11.6, Optimal Staging
% See Example #11.7 and Problem #11.6

% Also need functions:
% newton.m
% stepmass.m
% potential_dv.m

% INPUTS
% isp: array of ISPs for each stage of LV [seconds]
% e: array of structure ratios for each stage of LV
% v: burnout velocity [km/s]
% N: number of stages
% m_pl: mass of payload [kg]

% OUTPUTs
% eta: iterative solution needed to calculate everything else
% MF: array of optimum mass ratios of each stage (m_o/m_d)
% m_step: array of step masses of each stage (structure mass + propellant mass)
% m_o: total mass of vehicle [kg]
% m_S: array of structural masses of each stage [kg]
% m_P: array of propellant mass of each stage [kg]
% lam_total: overall payload fraction of entire LV (m_pl/m_o)
% lambda: array of payload ratios of each stage (m_pl/(m_p+m_S))
% check: all numbers in array should be positive to confirm local minimum
% was found

%% Display

% disp(['eta          = ', num2str(eta)])
% disp(['MF           = ', num2str(MF)])
% disp(['m_step       = ', num2str(m_step), ' kg'])
% disp(['m_o          = ', num2str(m_o), ' kg'])
% disp(['m_S          = ', num2str(m_S), ' kg'])
% disp(['m_P          = ', num2str(m_P), ' kg'])
% disp(['lam_total    = ', num2str(lam_total)])
% disp(['lambda       = ', num2str(lambda)])
% disp(['check        = ', num2str(check)])

%% Figures

figure
plot(m_pl,m_o(:,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_o(:,2),'LineWidth',2,'Color','r')
xlabel('Payload Mass [kg]')
ylabel('Total LV Mass (structure + fuel + payload) [kg]')
legend('Imaging','Comms')
title('2 stage total LV mass')
grid on
hold off

figure
subplot(1,2,1)
plot(m_pl,m_S(:,1,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_S(:,1,2),'LineWidth',2,'Color','r')
title('Structure Mass of First Stage')
xlabel('Payload Mass [kg]')
ylabel('Structure Mass [kg]')
legend('Imaging','Comms')
grid on
hold off
subplot(1,2,2)
plot(m_pl,m_S(:,2,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_S(:,2,2),'LineWidth',2,'Color','r')
title('Structure Mass of Second Stage')
xlabel('Payload Mass [kg]')
ylabel('Structure Mass [kg]')
legend('Imaging','Comms')
hold off
grid on

figure
subplot(1,2,1)
plot(m_pl,m_P(:,1,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_P(:,1,2),'LineWidth',2,'Color','r')
title('Propellant Mass of First Stage')
xlabel('Payload Mass [kg]')
ylabel('Prop Mass [kg]')
legend('Imaging','Comms')
hold off
grid on
subplot(1,2,2)
plot(m_pl,m_P(:,2,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_P(:,2,2),'LineWidth',2,'Color','r')
title('Propellant Mass of Second Stage')
xlabel('Payload Mass [kg]')
ylabel('Prop Mass [kg]')
legend('Imaging','Comms')
hold off
grid on

figure
subplot(1,2,1)
plot(m_pl,m_S(:,1,1)+m_P(:,1,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_S(:,1,2)+m_P(:,1,2),'LineWidth',2,'Color','r')
xlabel('payload mass [kg]')
ylabel('structure + prop [kg]')
title('Structure + Prop 1st Stage')
legend('Imaging','Comms')
hold off
grid on
subplot(1,2,2)
plot(m_pl,m_S(:,2,1)+m_P(:,2,1),'LineWidth',2,'Color','b')
hold on
plot(m_pl,m_S(:,2,2)+m_P(:,2,2),'LineWidth',2,'Color','r')
xlabel('payload mass [kg]')
ylabel('structure + prop [kg]')
title('Structure + Prop 2nd Stage')
legend('Imaging','Comms')
hold off
grid on

%% Test Cases
% clc
% mu = 398600;
% alt = 625;
% ra = 6378 + alt;
% ecc = .7;
% rp = ra*((1-ecc)/(1+ecc));
% a = (rp+ra)/2;
% r_out = 6378+150;
% 
% % payload mass (kg)
% m_pl = 100;
% % number of stages (1 - 5)
% N = 2; 
% % Specific Impulse of each stage (seconds)
% isp = [300 235];
% % Structure Ratios of each stage 
% e = [.2 .3];
% % Burnout Velocity (km/s)
% % v = sqrt(-mu/(2*a) + mu/r_out);
% v = 6.2;
% % Guess for newton's iteration (.1 - 1)
% eta_guess = .7;
% 
% [eta,MF,m_step,m_o,m_S,m_P,lam_total,lambda,check] = OptStaging(isp,e,v,N,m_pl,eta_guess);
% 
% disp(['eta          = ', num2str(eta)])
% disp(['MF           = ', num2str(MF)])
% disp(['m_step       = ', num2str(m_step), ' kg'])
% disp(['m_o          = ', num2str(m_o), ' kg'])
% disp(['m_S          = ', num2str(m_S), ' kg'])
% disp(['m_P          = ', num2str(m_P), ' kg'])
% disp(['lam_total    = ', num2str(lam_total)])
% disp(['lambda       = ', num2str(lambda)])
% disp(['check        = ', num2str(check)])