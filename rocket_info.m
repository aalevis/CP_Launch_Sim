% rocket info input
clear all
close all
clc

% Constants
deg = pi/180;  % ...Convert degrees to radians

% Rocket parameters (default Vega)

[ t_l,launch_char ] = model_launch('vega.txt','no') ;


v = launch_char(:,1)*1.e-3;   % ...Velocity (km/s)
gamma = launch_char(:,2)/deg; % ...Flight path angle (degrees)
x = launch_char(:,3)*1.e-3;   % ...Downrange distance (km)
h = launch_char(:,4)*1.e-3;   % ...Altitude (km)
vD = -launch_char(:,5)*1.e-3; % ...Velocity loss due to drag (km/s)
vG = -launch_char(:,6)*1.e-3; % ...Velocity loss due to gravity (km/s)
