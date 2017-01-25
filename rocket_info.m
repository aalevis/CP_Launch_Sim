% rocket info input
clearvars
close all
clc

%% Rocket Simulation Master

% ----- To Do -----
%
%

% Constants
deg = pi/180;  % ...Convert degrees to radians

%% Rocket input parameters

m_pl = 5000; % kg,  Payload Mass
h_turn = 130; % ...Height at which pitchover begins (m)
gamma_turn = 89.6; % deg, initial flight path angle at pitchover   --- delta4_54: 89  F9: 89.7 ---

[ t,launch_char ] = model_launch('f9.txt', m_pl, h_turn, gamma_turn);

% ----- Format for text file ----- 
% External Boosters   m_o  m_p  Thrust Isp  diam  t_burn  num_boost   
% 1st stage           m_o  m_p  Thrust Isp  diam  t_burn  num_boost
% S2                  m_o  m_p  Thrust Isp  diam  t_burn  num_boost
% S3                  m_o  m_p  Thrust Isp  diam  t_burn  num_boost
% S4                  m_o  m_p  Thrust Isp  diam  t_burn  num_boost

%% Results

v = launch_char(:,1)*1.e-3;   % ...Velocity (km/s)
gamma = launch_char(:,2)/deg; % ...Flight path angle (degrees)
x = launch_char(:,3)*1.e-3;   % ...Downrange distance (km)
h = launch_char(:,4)*1.e-3;   % ...Altitude (km)
vD = -launch_char(:,5)*1.e-3; % ...Velocity loss due to drag (km/s)
vG = -launch_char(:,6)*1.e-3; % ...Velocity loss due to gravity (km/s)
dV = vD + vG + v; % km/s  Total Mission dV 

% accel
a = zeros(1, length(launch_char));
a(1) = 0; % m/s^2
for i = 2:length(launch_char)
    a(i) = (v(i)*1e3 - v(i - 1)*1e3)/(t(i) - t(i - 1)); % m/s^2
end

% gamma_dot
gamma_dot = zeros(1, length(launch_char));
gamma_dot(1) = 0; % m/s^2
for i = 2:length(launch_char)
    gamma_dot(i) = (gamma(i) - gamma(i - 1))/(t(i) - t(i - 1)); % deg/s
end

figure
subplot(1,2,1)
plot(t, gamma, 'Linewidth', 4)
ob = title('Flight Path Angle vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
xlabel('Time (s)'), ylabel('Flight Path Angle (degrees)'), grid on

subplot(1,2,2)
plot(t, gamma_dot, 'Linewidth', 4)
ob = title('Flight Path Angle Derivative  vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
xlabel('Time (s)'), ylabel('Flight Path Angle Derivative (deg/s)'), grid on

figure
subplot(1,2,1)
plot(t, vD, 'Linewidth', 4), hold on
plot(t, vG, 'Linewidth', 4)
ob = title('Delta-V Losses vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
xlabel('Time (s)'), ylabel('Delta-V Loss (m/s)'), grid on
legend('Drag', 'Gravity', 'Location', 'Northwest')

subplot(1,2,2)
plot(t, dV, 'Linewidth', 4)
ob = title('Mission Delta-V vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
xlabel('Time (s)'), ylabel('Delta-V (k/s)'), grid on

fig = figure('Position', [400, 300, 900, 700]);
subplot(2,2,1)
plot(x,h, 'Linewidth', 4)
ob = title('Altitude vs. Downrange Distance');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
ylabel('Altitude (km)'), xlabel('Downrange Distance (km)'), grid on

subplot(2,2,2)
plot(t,h, 'Linewidth', 4)
ob = title('Altitude vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
ylabel('Altitude (km)'), xlabel('Time (s)'), grid on

subplot(2,2,3)
plot(t,v, 'Linewidth', 4)
ob = title('Velocity vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
ylabel('Velocity (km/s)'), xlabel('Time (s)'), grid on

subplot(2,2,4)
plot(t,a, 'Linewidth', 4)
ob = title('Acceleration vs. Time');
set(ob, 'FontSize', 20), set(gca, 'FontSize', 18)
ylabel('Acceleration (m/s^2)'), xlabel('Time (s)'), grid on



