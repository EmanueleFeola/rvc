clc
clear

%% helicoidal traj
center = [0 0 0]';
ro = 10; % radius
du_deg = 15;
du_rad = deg2rad(du_deg);
lambda = 1; % if 0 we have circular traj

u = 0:du_rad:10*pi;
path = [center(1) + ro .* cos(u);
        center(2) + ro .* sin(u);
        center(3) + lambda .* u]; 


T = [-ro .* sin(u);
    ro .* cos(u);
    lambda * ones(size(u))];
T = T ./ norm(T);

N = [-ro .* cos(u);
    -ro .* sin(u);
    zeros(size(u));];
N = N ./ norm(N);

B = cross(T, N);

figure;
plot3(path(1, :), path(2, :), path(3, :));
hold on;
quiver3(path(1, :), path(2, :), path(3, :), T(1, :), T(2, :), T(3, :));
% quiver3(path(1, :), path(2, :), path(3, :), N(1, :), N(2, :), N(3, :));
% quiver3(path(1, :), path(2, :), path(3, :), B(1, :), B(2, :), B(3, :));

T_vector_norm = norm(T(:, 1)); % norm of vector in first position (just to check one)
N_vector_norm = norm(N(:, 1)); % norm of vector in first position (just to check one)
T_vector_norm
N_vector_norm
