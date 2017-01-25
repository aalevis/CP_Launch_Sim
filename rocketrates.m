function [ dydt ] = rocketrates( t , y, t_burn, m_i, m_dot, Thrust, A, g0, Re, CD, h_turn, gamma_check, i, last_stage)
% Calculates the time rates df/dt of the variables f(t)
% in the equations of motion of a gravity turn trajectory.
%-------------------------

%...Initialize dfdt as a column vector:
dydt = zeros(6,1);
v = y(1);     % ...Velocity
gamma = y(2); % ...Flight path angle
x = y(3);     % ...Downrange distance
h = y(4);     % ...Altitude
vD = y(5);    % ...Velocity loss due to drag
vG = y(6);    % ...Velocity loss due to gravity
%...When time t exceeds the burn time, set the thrust
% and the mass flow rate equal to zero:
if t < t_burn
    m = m_i - m_dot*t; % ...Current vehicle mass
    T = Thrust;       % ...Current thrust
else
    m = m_i - m_dot*t_burn; % ...Current vehicle mass
    T = 0;                % ...Current thrust
end

g = g0/(1 + h/Re)^2; % ...Gravitational variation

% Drag
[ ~,rho ] = atmosnrlmsise00( h,0,0,2017,1,0,'none' );
D = 1/2 * rho(6)*v^2 * A * CD; % ...Drag [Equation 11.1]

%...Define the first derivatives of v, gamma, x, h, vD and vG
% ("dot" means time derivative):

%...Start the gravity turn when h = hturn:
if h <= h_turn
    v_dot = T/m - D/m - g;
    gamma_dot = 0;
    x_dot = 0;
    h_dot = v;
    vG_dot = -g;
else
    v_dot = T/m - D/m - g*sin(gamma); % m/s^2  ...Equation 11.6 
    x_dot = Re/(Re + h)*v*cos(gamma);              % ...Equation 11.8(1)
    h_dot = v*sin(gamma);                          % ...Equation 11.8(2)
    vG_dot = -g*sin(gamma);                        % ...Gravity loss rate
    
    if gamma_check == 0
         gamma_dot = -1/v*(g - v^2/(Re + h))*cos(gamma);% ...Equation 11.7  (rad/s)
    elseif gamma_check == 1
        gamma_dot = 0; 
    end
    
end


vD_dot = -D/m; % ...Drag loss rate

%...Load the first derivatives of f(t) into the vector dfdt:
dydt(1) = v_dot;
dydt(2) = gamma_dot;
dydt(3) = x_dot;
dydt(4) = h_dot;
dydt(5) = vD_dot;
dydt(6) = vG_dot;

end
