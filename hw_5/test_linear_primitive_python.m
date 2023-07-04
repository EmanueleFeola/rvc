Ts = 0.01; % sampling time
points = [[0 0 0]', [1 0 0]', [2 1 0]', [2 1 2]', [2 0 2]']; % forward path
pi = points(:, 1);
pf = points(:, 2);
[time, p, p_dot, p_ddot, p_dddot] = linear_primitive(pi, pf, Ts);

p