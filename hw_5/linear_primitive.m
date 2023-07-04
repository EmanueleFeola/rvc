function [time, p, p_dot, p_ddot, p_dddot] = linear_primitive(pi, pf, Ts)
s = [0:Ts:norm(pf - pi)];    
p = pi + s .* (pf - pi) / norm(pf - pi);
p_dot = ones(size(p)) .* (pf - pi) / norm(pf - pi); % derivata di p wrt s
p_ddot = zeros(size(p));
p_dddot = zeros(size(p));
time = s;
end