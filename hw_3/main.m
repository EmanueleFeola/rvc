clc;
clear;

%% Planner parameters
Ts = 0.001; % sampling time

qi_dot = 0;
qf_dot = 0;

num_points = 50;
x = linspace(0, 2*pi, num_points);
points = zeros(num_points, 2);
for i=1:num_points
    points(i, 1) = 2.3 * cos(3.0 * x(i)) + 1.2 * sin(4.5 * x(i)) + cos(1.92 * x(i)) + 2*rand(1);
end

points(:, 2) = round(x*2, 1);
figure(); scatter(points(:, 2), points(:, 1));
return;

%% Interpolating polynomials with computed velocities at path points and imposed velocity at initiala and final points
tic
[time, q, q_dot, q_ddot] = spline_1(Ts, points, qi_dot, qf_dot);
toc
tic
traj_plotter("spline_1", points, time, q, q_dot, q_ddot);
toc
return
%% Interpolating polynomials with continuous accelerations at path points and imposed velocity at initial and final points
[time, q, q_dot, q_ddot] = spline_2(Ts, points, qi_dot, qf_dot);
traj_plotter("spline_2", points, time, q, q_dot, q_ddot);
return
%% Compute cubic splines based on the accelerations with assigned initial and final velocities
[time, q, q_dot, q_ddot] = spline_3(Ts, points, qi_dot, qf_dot);
traj_plotter("spline_3", points, time, q, q_dot, q_ddot);
return;

%% spline_smoothing
rng('default'); % repeatable random numbers
rng(1); % repeatable random numbers

w_inv = randsrc(1,num_points, [0,1 ; 0.1,0.9]);

all_q = [];

mus = linspace(0.1, 1, 5);
for i = 1:size(mus, 2)
    mu = mus(i);
    [smoothed_points] = spline_smoothing(points, mu, w_inv);
    [time, q, q_dot, q_ddot] = spline_3(Ts, smoothed_points, qi_dot, qf_dot);
    traj_plotter_extended(num2str(mu), points, time, q, q_dot, q_ddot, w_inv);

    all_q = [all_q; q];
    pause(1);
end


figure;
hold on;
for i = 1:size(points, 1)
    if w_inv(i) == 0
        plot(points(i, 2), points(i, 1), 'black.','MarkerSize',12, 'HandleVisibility','off');
    else
        plot(points(i, 2), points(i, 1), 'green.','MarkerSize',12, 'HandleVisibility','off');
    end
end

for i = 1:size(mus, 2)
    plot(time, all_q(i, :), 'DisplayName', ['mu ' num2str(mus(i))]);
end
legend show;

export_fig(strcat('E:\uni\magistrale\rvc\lab\images\', 'mu_diversi'), '-pdf', '-transparent');
close;