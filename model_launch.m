function [ t1,f1 ] = model_launch(fid, m_pl, h_turn, gamma_turn)

deg = pi/180;       % ...Convert degrees to radians
g0 = 9.81;          % ...Sea-level acceleration of gravity (m/s^2)
Re = 6378e3;        % ...Radius of the earth (m)
CD = 0.5;           % ...Drag coefficient (assumed constant)
t0 = 0;             % ...Initial time for the numerical integration

% Load Roacket info
data = load(fid);

m_o     = data(:, 1)   ; %[kg] total mass of each stage (before any burns)
m_p     = data(:, 2)   ; %[kg] propellant mass
Thrust  = data(:, 3)   ; %[N]
Isp     = data(:, 4)   ; %[s]
diam    = data(:, 5)   ; %[m]
t_burn  = data(:, 6)   ; %[s]
num_boosters =  data(:, 7) ; % number of boosters (while each "core stage" should only have 1 booster, "external stages" may have more) 

% ----- Mass Flow Rate ----- Allows us to calc MF_p (with an external stage)

for i = 1:length(m_o) % 1 external stage, no limit on core stages  
    
    if i == 1 && m_o(1) > 0 % external stage
         m_dot(1) = .95*(num_boosters(1)*m_p(1)/t_burn(1) + m_p(2)/t_burn(2));     % kg/s Assume use 95% of propellant
         
    else                    % not an external stage
         m_dot(i) = .95*m_p(i)/t_burn(i); % kg/s

    end    
end

% ----- Build ode inputs from txt file info ----- 

% don't need MF_p or MF, nice to have though

for i = 1:length(m_o) % all stages
    
    if m_o(1) == 0 || (m_o(1) > 0 && i > 2) % no external stage  OR  vehicle has external stage & these are after the 1st stage (unlimited)
        
        m_i(i) = sum(m_o(i:end)) + m_pl; % initial mass for the stack starting with this stage
        MF_p(i) = m_p(i)/(sum(m_o(i:end)) + m_pl); % convert from MF_prop_individual to MF_prop_stage
        MF(i) = 1/(1-MF_p(i));     % ...Mass ratio (m_o/m_d)
%         m_d(i) = m_o(i)/MF(i);     % ...Burnout mass (kg)   
        A(i) = pi/4*(diam(i))^2;    % ...Frontal area (m^2)
        T(i) = Thrust(i); % N
        
    elseif m_o(1) > 0 && i == 1 % vehicle has external stage & this is that external stage in tandem with S1
        
        m_i(1) = num_boosters(1)*m_o(1) + sum(m_o(2:end)) + m_pl; % initial mass for the stack starting with this stage
        MF_p(1) = (m_p(1) + m_dot(2)*t_burn(1))/(sum(m_o) + m_pl);     
        MF(1) = 1/(1-MF_p(1));     % ...Mass ratio (m_o/m_d)
        A(1) = num_boosters(1)*pi/4*(diam(1))^2 + pi/4*(diam(2))^2;   % ...Frontal area (m^2)
        T(1) = num_boosters(1)*Thrust(1) + Thrust(2); % N

    elseif m_o(1) > 0 && i == 2 % vehicle has external stage & this is the 1st core stage (S1)
        
        m_i(2) = sum(m_o(i:end)) - m_dot(2)*t_burn(1) + m_pl; % initial mass for the stack starting with this stage
        MF_p(2) = (m_p(2) - m_dot(2)*t_burn(1))/(m_o(2) - m_dot(2)*t_burn(1) + sum(m_o(3:end)) + m_pl); 
        MF(2) = 1/(1-MF_p(1));     % ...Mass ratio (m_o/m_d)
        A(2) = pi/4*(diam(2))^2;   % ...Frontal area (m^2)  
        T(2) = Thrust(2); % N

    end
end

% ...Initial conditions:
v0 = 0;             % ...Initial velocity (m/s)
gamma0 = 90*deg; % ...Initial flight path angle (rad)
x0 = 0;             % ...Initial downrange distance (km)
h0 = 0;             % ...Initial altitude (km)
vD0 = 0;            % ...Initial value of velocity loss due to drag (m/s)
vG0 = 0;            % ...Initial value of velocity loss due to gravity (m/s)

%...Initial conditions vector:
f0 = [v0; gamma0; x0; h0; vD0; vG0];

gamma_check = 0; % when gamma_check = 1, gamma is 0
t1 = 0;
f1 = 0;


% Simulate each stage, one at a time (external boosters fire at launch with
%                                   S1)

for i = 1:length(m_o)
    if m_o(i) == 0
        continue
    end
    
    if  i == 2 && m_o(1) > 0 % this is the 1st Core Stage when there is an external stage
      tspan = [t0, t_burn(i) - t_burn(1)];           % ...Range of integration
    
    else
      tspan = [t0, t_burn(i)];           % ...Range of integration
      
    end

    % ode
        options = odeset('Abstol', 1e-8, 'Reltol', 1e-8,'Events', @Rocket_Events);

        [t, f, ~, ~, ~] = ode45(@rocketrates, tspan, f0, options, t_burn(i), m_i(i), m_dot(i), ...
            T(i), A(i), g0, Re, CD, h_turn, gamma_check, i, length(m_o));  
        
     % combine stage results
        
    if (i == 1 && m_o(1) > 0) || (i == 2 && m_o(1) == 0)  % (external stage) OR (first stage when there are no external stages)
        t1 = t ;
        f1 = f ;
    else
        t1 = [t1; t + t1(end)] ;
        f1 = [f1; f] ;
    end
    
    f0 = f1(end,:) ;
    
    % Handle Events (h_turn control, gamma control)
    
    if round(f0(4)) == h_turn % currently at h_turn, input inial gamma_turn and resume stage
        
            tspan = [0 t_burn(i) - t(end)];  % calc new tspan
            f0 = [f0(1); gamma_turn*deg; f0(3); f0(4); f0(5); f0(6)];
            m_i(i) = m_i(i) - t(end)*m_dot(i);
            
            options = odeset('Abstol', 1e-8, 'Reltol', 1e-8,'Events', @gamma_event);

            [t, f, ~, ~, ~] = ode45(@rocketrates, tspan, f0, options, t_burn(i), m_i(i), m_dot(i),...
                 T(i), A(i), g0, Re, CD, h_turn, gamma_check, i, length(m_o));
             
           
            % combine stage results
        
            if (i == 1 && m_o(1) > 0) || (i == 2 && m_o(1) == 0)  % (external stage) OR (first stage when there are no external stages)
                t1 = t ;
                f1 = f ;
            else
                t1 = [t1; t + t1(end)] ;
                f1 = [f1; f] ;   
            end

            f0 = f1(end,:) ;
        
    elseif round(f0(2)) == 0  %  flight path angle ~ 0, make gamma_dot = 0 and finish stage
            gamma_check = 1;
            tspan = [0 t_burn(i) - t(end)];  % calc new tspan
            m_i(i) = m_i(i) - t(end)*m_dot(i);
            
            options = odeset('Abstol', 1e-8, 'Reltol', 1e-8,'Events', @gamma_event);

            [t, f, ~, ~, ~] = ode45(@rocketrates, tspan, f0, options, t_burn(i), m_i(i), m_dot(i),...
                 T(i), A(i), g0, Re, CD, h_turn, gamma_check, i, length(m_o));
               
            % combine stage results
        
            if (i == 1 && m_o(1) > 0) || (i == 2 && m_o(1) == 0)  % (external stage) OR (first stage when there are no external stages)
                t1 = t ;
                f1 = f ;
            else
                t1 = [t1; t + t1(end)] ;
                f1 = [f1; f] ;   
            end

            f0 = f1(end,:) ;
            
    end
    
end


end
