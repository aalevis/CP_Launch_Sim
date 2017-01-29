function [value, isterminal, direction] = orbit_event(t, y, t_burn, m_i, m_dot,...
            T, A, g0, Re, CD, h_turn, h_targ_check, h_targ, i, num_stages)

value = y(4) - h_targ; % m, altitude
isterminal = 1; % stop integration
direction = 1; % approach with a positive slope

end
