function [time, p, p_dot, p_ddot, p_dddot] = circular_primitive(pi,pf,c,circle_axis,Ts,sign)
ro = norm(pi - c); % radius
a = (pi - c)/ro; % vector from initial point to circle center
b = (pf - c)/ro; % vector from final point to circle center
theta = acos(dot(a,b) / (norm(a) * norm(b))); % find angle between 2 vectors: theta = cos-1 [(a·b) / (|a| |b|)]
u = [0:Ts:theta]; % angles from start to end
u = u .* sign;
circle_axis = circle_axis / norm(circle_axis);
R = [pi-c, cross(circle_axis, pi-c), circle_axis]; % rot matrix from sigma' to sigma

p = c + R * [ro*cos(u); ro*sin(u); zeros(size(u))];
p_dot = R * [-ro*sin(u); ro*cos(u); zeros(size(u))];
p_ddot = R * [-ro*cos(u); -ro*sin(u); zeros(size(u))];
p_dddot = R * [ro*sin(u); -ro*cos(u); zeros(size(u))];

time = u;
end