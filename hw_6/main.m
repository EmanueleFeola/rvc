% Let p1, p2, p3 three points on a sphere of center p0 and radius R. Design the
% trajectory such that (1) the EE will pass through the three points along the shortest
% path, and (2) the z axis of the EE is always orthogonal to the sphere
clc;
clear;
rng('shuffle');

%% define sphere (p0, r) and 3 random points
Ts = 0.1;
r = 0.02;
% p0 = [-0.07 1.449 -0.245]';
p0 = [0.085 0.8 -0.166]';
[X,Y,Z] = sphere(50);
X = X * r + p0(1);
Y = Y * r + p0(2);
Z = Z * r + p0(3);

n_points = 2;
theta = rand([1, n_points]) * pi;
phi = rand([1, n_points]) * 2*pi;
points = p0 + [r * sin(theta).*cos(phi)
          r * sin(theta).*sin(phi)
          r * cos(theta)         ];
 
% points = [    0.0235   -0.0744   -0.0078
%     1.5960    1.4480    1.3817
%    -0.1467   -0.0451   -0.4228];

%% traj
p_total = [];
ff_T = [];
ff_N = [];
ff_B = [];

for i=1:n_points-1
    pi = points(:, i);
    pf = points(:, i+1);
    [time, p, frenet_frames] = circular_primitive_2(pi,pf,p0,Ts);  
    p_total = [p_total p];
    ff_T = [ff_T frenet_frames.T];
    ff_N = [ff_N frenet_frames.N];
    ff_B = [ff_B frenet_frames.B];
end
% pi = points(:, end);
% pf = points(:, 1);
% circle_axis = cross(pi, pf);
% [time, p, frenet_frames] = circular_primitive_2(pi,pf,p0,Ts);  
% p_total = [p_total p];
% ff_T = [ff_T frenet_frames.T];
% ff_N = [ff_N frenet_frames.N];
% ff_B = [ff_B frenet_frames.B];

%% plot sphere, points, traj
h = surf(X,Y,Z);
set(h, 'FaceAlpha', 0.2)
shading interp
axis equal
hold on;
scatter3(points(1,:), points(2,:), points(3,:), 60, 'filled', 'r');
plot3(p_total(1,:),p_total(2,:),p_total(3,:),'LineWidth',2,'color','g');

quiver3(p_total(1,:),p_total(2,:),p_total(3,:), ff_T(1,:),ff_T(2,:),ff_T(3,:),'Color','red');
quiver3(p_total(1,:),p_total(2,:),p_total(3,:), ff_N(1,:),ff_N(2,:),ff_N(3,:),'Color','green');
quiver3(p_total(1,:),p_total(2,:),p_total(3,:), ff_B(1,:),ff_B(2,:),ff_B(3,:),'Color','blue');

%% write csv file
quat_list = [];
for i=1:size(p_total, 2)
%     rot_matrix = [ff_T(:,i), ff_N(:,i), ff_B(:,i)];
    rot_matrix = [ff_B(:,i), ff_T(:,i), ff_N(:,i)];
    quat_orient = rotm2quat(rot_matrix);
    quat_list = [quat_list; quat_orient];
end

csvwrite('myFile.csv',[p_total',quat_list]);
