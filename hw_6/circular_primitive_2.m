function [time, p, frenet_frames] = circular_primitive_2(pi,pf,c,Ts)
% differenza con circular_primitive
% - il circle_axis non viene passato come param: viene calcolato dentro questa funzione
% - non c'è bisogno del parametro sign (indifferente)

ro = norm(pi - c); % radius
a = (pi - c) / ro; % vector from initial point to circle center
b = (pf - c) / ro; % vector from final point to circle center
circle_axis = cross(a, b);
circle_axis = circle_axis / norm(circle_axis);

theta = acos(dot(a,b) / (norm(a) * norm(b))); % find angle between 2 vectors: theta = cos-1 [(a·b) / (|a| |b|)]
u = [0:Ts:theta]; % angles from start to end

% R = [pi-c, cross(circle_axis, pi-c), circle_axis]; % rot matrix from sigma' to sigma
R = [a, cross(circle_axis, a), circle_axis]; % rot matrix from sigma' to sigma
p = c + R * [ro*cos(u); ro*sin(u); zeros(size(u))];

%% frenet frames
T = R * [-ro .* sin(u);
    ro .* cos(u);
    zeros(size(u))];
T = T ./ norm(T);

N = R * [-ro .* cos(u);
    -ro .* sin(u);
    zeros(size(u));];
N = N ./ norm(N);

B = cross(T, N);
B = B ./ norm(B);

frenet_frames.T = T;
frenet_frames.N = N;
frenet_frames.B = B;

time = u;
end