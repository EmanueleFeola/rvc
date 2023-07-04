function [t,q,q_dot,q_ddot,q_dddot,q_ddddot] = poly5_traj(Ts,ti,tf,qi,qf,qi_dot,qf_dot,qi_ddot,qf_ddot)
Q=[qi;qi_dot;qi_ddot;qf;qf_dot;qf_ddot];
t=ti:Ts:tf;
dt=tf-ti;

%% compute coefficients
X=[ 1,      0,      0,      0,      0,      0;
    0,      1,      0,      0,      0,      0;
    0,      0,      2,      0,      0,      0;
    
    1,      dt,     dt^2,   dt^3,   dt^4,   dt^5;
    0,      1,      2*dt,   3*dt^2, 4*dt^3, 5*dt^4;
    0,      0,      2,      6*dt,   12*dt^2,20*dt^3];

A = inv(X) * Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);
a4=A(5);
a5=A(6);

%% compute trajectory
syms t_sym
q=a5*(t_sym-ti).^5+a4*(t_sym-ti).^4+a3*(t_sym-ti).^3+a2*(t_sym-ti).^2+a1*(t_sym-ti)+a0;
q_dot=diff(q, t_sym);
q_ddot=diff(q_dot, t_sym); 
q_dddot=diff(q_ddot, t_sym); 
q_ddddot=diff(q_dddot, t_sym);

q = subs(q, t_sym, t);
q_dot = subs(q_dot, t_sym, t);
q_ddot = subs(q_ddot, t_sym, t);
q_dddot = subs(q_dddot, t_sym, t);
q_ddddot = subs(q_ddddot, t_sym, t);

end