function [t,q,q_dot,q_ddot,q_dddot] = poly3_traj(Ts,ti,tf,qi,qf,qi_dot,qf_dot)
Q = [qi;qi_dot;qf;qf_dot];
dt = tf-ti;
t = ti:Ts:tf;

%% compute coefficients
% X = [1,ti,ti^2,ti^3;
%     0,1,2*ti,3*ti^2;
%     1,dt,dt^2,dt^3;
%     0,1,2*dt,3*dt^2];

% come quella commentata, però ti = 0
X = [1,0,0,0;
    0,1,0,0;
    1,dt,dt^2,dt^3;
    0,1,2*dt,3*dt^2];

A = inv(X) * Q;

a0 = A(1);
a1 = A(2);
a2 = A(3);
a3 = A(4);

%% compute trajectory
syms t_sym
q = a3*(t_sym-ti).^3+a2*(t_sym-ti).^2+a1*(t_sym-ti)+a0;
q_dot = diff(q, t_sym); %3*a3*(t-ti).^2+2*a2*(t-ti)+a1;
q_ddot = diff(q_dot, t_sym); %6*a3*(t-ti)+2*a2;
q_dddot = diff(q_ddot, t_sym); %6*a3*(t-ti)+2*a2;

q = subs(q, t_sym, t);
q_dot = subs(q_dot, t_sym, t);
q_ddot = subs(q_ddot, t_sym, t);
q_dddot = subs(q_dddot, t_sym, t);

end