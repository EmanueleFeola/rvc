% implement 3rd/cubic, 5th, 7th order polynomials
% for qi > qf and qi < qf
clc;
clear;

%% Planner parameters
Ts = 0.01; % sampling time

ti = 5;  
tf = 10; 

qi = 10;
qf = 2; 

qi_dot = 0; 
qf_dot = 5; 

qi_ddot = 2.5; 
qf_ddot = 0;

qi_dddot = 0; 
qf_dddot = 5;

points = [qi, ti; qf, tf];

%% cubic
% 4 constraints: initial/final position and velocity
[time, q, q_dot, q_ddot, q_dddot] = poly3_traj(Ts, ti, tf, qi, qf, qi_dot, qf_dot);
traj_plotter('poly3_traj', points, time, q, q_dot, q_ddot,q_dddot);

% 5th
% 6 constraints: initial/final position, velocity and acceleration
[time, q, q_dot, q_ddot, q_dddot] = poly5_traj(Ts, ti, tf, qi, qf, qi_dot, qf_dot, qi_ddot, qf_ddot);
traj_plotter('poly5_traj', points, time, q, q_dot, q_ddot, q_dddot);

%% 7th
% 8 constraints: initial/final position, velocity, acceleration, jerk
[time, q, q_dot, q_ddot, q_dddot, q_ddddot] = poly7_traj(Ts, ti, tf, qi, qf, qi_dot, qf_dot, qi_ddot, qf_ddot, qi_dddot, qf_dddot);
traj_plotter('poly7_traj', points, time, q, q_dot, q_ddot, q_dddot, q_ddddot);