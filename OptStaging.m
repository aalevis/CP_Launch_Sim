function [eta,MF,m_step,m_o,m_S,m_P,lam_total,lambda,check] = OptStaging(isp,e,v,N,m_pl,eta_guess)
%% Optimal Staging for a tandem stacked LV
% From Curtis 11.6, Example 11.7

% Also need functions:
% newton.m
% stepmass.m

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

syms eta
%% Gravity
g = .00981; % km/s

%% Calculate charateristic velocity from Isp

for i = 1:N;
    c(i) = isp(i)*g;
end

%% A

for i = 1:N
   alpha(i) =  c(i)*log(c(i)*eta-1);
end

A = sum(alpha);

%% B

for i = 1:N
   beta(i) = c(i);
end

B = log(eta)*sum(beta);

%% C

for i = 1:N
    charlie(i) = c(i)*log(c(i)*e(i));
end

C = sum(charlie);

%% Full equation and derivative

ABC = A - B - C - v;
dABC = diff(ABC,eta);

%% Iterative Solution
eta = eta_guess;
[eta] = newton(ABC,dABC,eta);

%% Optimum Mass Ratio, MF

for i = 1:N
   MF(i) = (c(i)*eta-1) /(c(i)*e(i)*eta);
end

%% Step Masses of each stage

[m_step] = stepmass(N,MF,e,m_pl);

%% Total Mass of Vehicle

m_o = sum(m_step) + m_pl;

%% Structural Mass of each stage

for i = 1:N
   m_S(i) = e(i)*m_step(i); 
end

%% Propellant Mass of Each Stage

for i = 1:N
    m_P(i) = m_step(i) - m_S(i);
end

%% Overall Mass Payload Fraction

lam_total = m_pl/m_o;

%% Payload Ratio of Each Stage

i = 1;
while i+1<=N
    lambda(i) = (sum(m_step(i+1:N)) + m_pl)/m_step(i);
    i = i + 1;
end
lambda(end+1) = m_pl/m_step(N);

%% Local Minimum Check

for i = 1:N
    % must be greater than 1
   check(i) = eta*c(i)*(e(i)*MF(i)-1)^2 + 2*e(i)*MF(i) - 1; 
end

end