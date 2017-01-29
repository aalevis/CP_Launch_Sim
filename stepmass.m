function [m_step] = stepmass(N,MF,e,m_pl)
% find the mass of each stage
% INPUTS
% N = number of stages (1 - 5)
% MF = mass fraction
% e = structural ratio
% m_pl = mass of payload

% OUTPUTS
% m_step: array of step masses of each stage (structure mass + propellant mass)

if N == 1
        m_step = ((MF(N)-1)/(1-MF(N)*e(N)))*m_pl;
elseif N == 2
        m_step(N) = ((MF(N)-1)/(1-MF(N)*e(N)))*m_pl;
        m_step(N-1) = ((MF(N-1)-1)/(1-MF(N-1)*e(N-1)))*(m_step(N)+m_pl);
elseif N == 3
        m_step(N) = ((MF(N)-1)/(1-MF(N)*e(N)))*m_pl;
        m_step(N-1) = ((MF(N-1)-1)/(1-MF(N-1)*e(N-1)))*(m_step(N)+m_pl);
        m_step(N-2) = ((MF(N-2)-1)/(1-MF(N-2)*e(N-2)))*(m_step(N-1)+m_step(N)+m_pl);
elseif N == 4
        m_step(N) = ((MF(N)-1)/(1-MF(N)*e(N)))*m_pl;
        m_step(N-1) = ((MF(N-1)-1)/(1-MF(N-1)*e(N-1)))*(m_step(N)+m_pl);
        m_step(N-2) = ((MF(N-2)-1)/(1-MF(N-2)*e(N-2))*(m_step(N-1)+m_step(N)+m_pl));
        m_step(N-3) = ((MF(N-3)-1)/(1-MF(N-3)*e(N-3)))*(m_step(N-2)+m_step(N-1)+m_step(N)+m_pl);
elseif N == 5
        m_step(N) = ((MF(N)-1)/(1-MF(N)*e(N)))*m_pl;
        m_step(N-1) =((MF(N-1)-1)/(1-MF(N-1)*e(N-1)))*(m_step(N)+m_pl);
        m_step(N-2) =((MF(N-2)-1)/(1-MF(N-2)*e(N-2)))*(m_step(N-1)+m_step(N)+m_pl);
        m_step(N-3) =((MF(N-3)-1)/(1-MF(N-3)*e(N-3)))*(m_step(N-2)+m_step(N-1)+m_step(N)+m_pl);
        m_step(N-4) =((MF(N-4)-1)/(1-MF(N-4)*e(N-4)))*(m_step(N-3)+m_step(N-2)+m_step(N-1)+m_step(N)+m_pl);
end

end

