function [value, isterminal, direction] = Rocket_Events(t, y, t_burn, m_i, m_dot,...
            T, A, g0, Re, CD, h_turn, h_targ_check, h_targ, i, num_stages) 
        
    % 1st fxn is h_turn (when h = h_turn), 2nd fxn is h_targ (when
    % h = h_targ)

value = [y(4) - h_turn, y(4) - h_targ]; % m  Altitude, % rad flight path angle
isterminal = [1, 1]; % stop integration
direction = [1, 1]; % approach h with a positive slope

end
