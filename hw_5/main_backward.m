clc;
clear;

%% Planner parameters
Ts = 0.001; % sampling time
p_total = [];
p_dot_total = [];
p_ddot_total = [];
p_dddot_total = [];

splines_p_total = [];
splines_p_dot_total = [];
splines_p_ddot_total = [];
splines_p_dddot_total = [];

points = [[0 0 0]', [1 0 0]', [2 1 0]', [2 1 2]', [2 0 2]']; % forward path
circle_centers = [[1 1 0]', [2 1 1]']; % forward path

points = flip(points, 2); % backward path
circle_centers = flip(circle_centers, 2); % backward path

%% 1 - linear primitive
pi = points(:, 1);
pf = points(:, 2);
[time, p, p_dot, p_ddot, p_dddot] = linear_primitive(pi, pf, Ts);
p_total = [p_total, p];
p_dot_total = [p_dot_total, p_dot];
p_ddot_total = [p_ddot_total, p_ddot];
p_dddot_total = [p_dddot_total, p_dddot];

%% 2 - circular primitive
pi = points(:, 2);
pf = points(:, 3);
c = circle_centers(:, 1);
circle_axis = [1 0 0]';
[time, p, p_dot, p_ddot, p_dddot] = circular_primitive(pi,pf,c,circle_axis,Ts);
p_total = [p_total, p];
p_dot_total = [p_dot_total, p_dot];
p_ddot_total = [p_ddot_total, p_ddot];
p_dddot_total = [p_dddot_total, p_dddot];

%% 3 - circular primitive
pi=points(:, 3);
pf=points(:, 4);
c = circle_centers(:, 2);
circle_axis = [0 0 1]';
[time, p, p_dot, p_ddot, p_dddot] = circular_primitive(pi,pf,c,circle_axis,Ts);
p_total = [p_total, p];
p_dot_total = [p_dot_total, p_dot];
p_ddot_total = [p_ddot_total, p_ddot];
p_dddot_total = [p_dddot_total, p_dddot];
% 
% %% 4 - linear primitive
% pi = points(:, 4);
% pf = points(:, 5);
% [time, p, p_dot, p_ddot, p_dddot] = linear_primitive(pi, pf, Ts);
% p_total = [p_total, p];
% p_dot_total = [p_dot_total, p_dot];
% p_ddot_total = [p_ddot_total, p_ddot];
% p_dddot_total = [p_dddot_total, p_dddot];

%% plot primitives
plot_points = [points, circle_centers];
traj_plotter('hw_5_motion_primitive', plot_points, p_total, p_dot_total, p_ddot_total, p_dddot_total);
return;
%% splines x
time_splines = [1, 2, 3, 4, 5];
qi_dot = p_dot_total(1, 1);
qf_dot = p_dot_total(1, end);
[time, q, q_dot, q_ddot, q_dddot] = spline_3(Ts, [points(1, :)', time_splines'], qi_dot, qf_dot);

splines_p_total = zeros(size(q)); % init array
splines_p_dot_total = zeros(size(q)); % init array
splines_p_ddot_total = zeros(size(q)); % init array
splines_p_dddot_total = zeros(size(q)); % init array

splines_p_total(1, :) = q;
splines_p_dot_total(1, :) = q_dot;
splines_p_ddot_total(1, :) = q_ddot;
splines_p_dddot_total(1, :) = q_dddot;
%% splines y
qi_dot = p_dot_total(2, 1);
qf_dot = p_dot_total(2, end);
[time, q, q_dot, q_ddot, q_dddot] = spline_3(Ts, [points(2, :)', time_splines'], qi_dot, qf_dot);
splines_p_total(2, :) = q;
splines_p_dot_total(2, :) = q_dot;
splines_p_ddot_total(2, :) = q_ddot;
splines_p_dddot_total(2, :) = q_dddot;

%% splines z
qi_dot = p_dot_total(3, 1);
qf_dot = p_dot_total(3, end);
[time, q, q_dot, q_ddot, q_dddot] = spline_3(Ts, [points(3, :)', time_splines'], qi_dot, qf_dot);
splines_p_total(3, :) = q;
splines_p_dot_total(3, :) = q_dot;
splines_p_ddot_total(3, :) = q_ddot;
splines_p_dddot_total(3, :) = q_dddot;

traj_plotter('hw_5_spline', plot_points, splines_p_total, splines_p_dot_total, splines_p_ddot_total, splines_p_dddot_total);