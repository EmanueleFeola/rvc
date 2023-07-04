% Implement in Matlab the Trapezoidal trajectory taking into account the different constraints
clc;
clear;

%% Planner parameters
Ts = 0.001; % sampling time

ti = 5;  
tf = 15;

qi = 0;
qf = 10;

points = [qi, ti; qf, tf];

%% given tc
% 4 constraints: initial/final position and velocity
% input: tc, i.e. the time from start to constant acc
tc = 2;
[time, q, q_dot, q_ddot] = trap_tc(Ts, qi, qf, ti, tf, tc);
traj_plotter('trap_tc', points, time, q, q_dot, q_ddot);

%% given qc_ddot
% 4 constraints: initial/final position and velocity
% input: qc_ddot, i.e. l'accelerazione da 0 a tc
qc_ddot = 3;
[time, q, q_dot, q_ddot] = trap_qc_ddot(Ts, qi, qf, ti, tf, qc_ddot);
traj_plotter('trap_qc_ddot', points, time, q, q_dot, q_ddot);

%% given qc_dot
% 4 constraints: initial/final position and velocity
% input: qc_dot, i.e. cruise velocity
% se qi > qf, allora qc_dot deve essere negativo!
qc_dot = 2; %1.5 trap, 2 tri
[time, q, q_dot, q_ddot] = trap_qc_dot(Ts, qi, qf, ti, tf, qc_dot);
traj_plotter(strcat('trap_qc_dot', num2str(qc_dot)), points, time, q, q_dot, q_ddot);

%% given qc_dot and qc_ddot
% 4 constraints: initial/final position and velocity
% input: qc_dot and qc_ddot, i.e. cruise velocity and acceleration from 0 to tc
% output: same as before, but also the time duration!
qc_dot = 6; %2 trap, 6 triangular
qc_ddot = 2;
[time, q, q_dot, q_ddot] = trap_qc_dot_qc_ddot(Ts, qi, qf, ti, -1, qc_dot, qc_ddot);
traj_plotter(strcat('trap_qc_dot_qc_ddot', num2str(qc_dot), num2str(qc_ddot)), points, time, q, q_dot, q_ddot);

%% given duration and qc_ddot_max
qi_dot = 1;
qf_dot = 0.5;
duration = 5;
qc_ddot_max = 2;
[time, q, q_dot, q_ddot] = trap_duration_qc_ddot_max(Ts, qi, qf, ti, duration, qi_dot, qf_dot, qc_ddot_max);
traj_plotter('trap_duration_qc_ddot_max', points, time, q, q_dot, q_ddot);

%% given qc_dot_max and qc_ddot_max (delta_T is output of planning)
qi_dot = 0.5;
qf_dot = 1;
qc_dot_max = 3;
qc_ddot_max = 4;
[time, q, q_dot, q_ddot] = trap_qc_ddot_max_qc_dot_max(Ts, qi, qf, ti, qi_dot, qf_dot, qc_dot_max, qc_ddot_max);
traj_plotter('trap_qc_ddot_max_qc_dot_max', points, time, q, q_dot, q_ddot);

%% multipoint traj
middle_points = [20,2; 50,4; 40,5]; % each row = {joint q, time t}
points = [qi, ti; middle_points; qf, tf]; % number of rows = number of total points

% qc_ddot_max = 5;
% trap_multipoint_1(points, Ts, qc_ddot_max);

qc_ddot_max = 20;
qc_dot_max = 15;
trap_multipoint_2(points, Ts, qc_ddot_max, qc_dot_max);
